"""
3D shoulder, elbow and wrist from MediaPipe landmarks and the organized point cloud.

One camera frame:
    1. read the shoulder, elbow and wrist points out of the cloud at the
       landmark pixels, and move each from the skin to the joint centre:
       the shoulder by a fixed offset, the elbow and wrist along the viewing ray
    2. reject the frame if a bone length is far from the subject's
    3. solve the shoulder and elbow angles (q0..q3) from the two bone directions
    4. add the q0/q1 offsets and rebuild the elbow from the corrected shoulder
       angles and the subject's upper-arm length; the wrist stays as measured
    5. publish shoulder, elbow, wrist on topics.arm_keypoints (camera frame)

A rejected frame publishes no keypoints, and false on topics.arm_keypoints_valid.
The node also broadcasts the shoulder frame (<arm_prefix>RightShoulder) the EKF
and the rendered arm hang off: it follows the measured shoulder for
SHOULDER_INIT_WINDOW_SEC, then stays where it was (ShoulderAnchor).
"""

import math
import time
from typing import Optional, Tuple

from arm_pose_estimation.node_main import required_parameter, run_node
from arm_pose_estimation.topic_watchdog import TopicWatchdog
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PolygonStamped, Pose, PoseArray
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Float64MultiArray
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# --- Calibration for this camera placement (fitted against motion capture) ---
# Acromion -> glenohumeral joint centre, in the camera optical frame (m).
SHOULDER_OFFSET = np.array([0.05, -0.05, 0.0])
# Skin -> joint centre along the viewing ray, away from the camera (m).
DEPTH_OFFSETS = {'right_elbow': 0.04, 'right_wrist': 0.02}
# Added to elv_angle and shoulder_elv before the elbow is rebuilt (rad).
Q0_OFFSET = math.radians(-5.0)
Q1_OFFSET = math.radians(-5.0)

# --- Frame processing ---
SAMPLE_RADIUS_PX = 2           # half-width of the pixel patch averaged at a landmark
MIN_CONFIDENCE = 0.5           # landmark visibility needed
MIN_VALID_SAMPLES = 3          # valid depth samples needed in the patch
MAX_PROCESSING_RATE_HZ = 15.0  # clouds processed per second, on the sensor clock
SEGMENT_LENGTH_RATIO = (0.60, 1.25)  # accepted bone length / subject's length
SHOULDER_INIT_WINDOW_SEC = 5.0
MARKER_RADIUS = 0.02
MARKER_LIFETIME_SEC = 0.5

# The solve runs in the shoulder frame; the cloud and the output are in the camera optical frame.
CAMERA_TO_SHOULDER_ROTATION = np.array([
    [0.0, 0.0, -1.0],   # x_shoulder = -z_camera
    [0.0, -1.0, 0.0],   # y_shoulder = -y_camera
    [-1.0, 0.0, 0.0],   # z_shoulder = -x_camera
])
SHOULDER_TO_CAMERA_ROTATION = CAMERA_TO_SHOULDER_ROTATION.T
# Orientation of the broadcast shoulder frame in the camera optical frame (x, y, z, w).
SHOULDER_FRAME_QUATERNION = R.from_euler('xyz', [3.14, 1.57, 0.0]).as_quat()

# Shoulder and elbow of right_arm.urdf.xacro (OpenSim shoulder).
SHOULDER_AXIS = np.array([0.0048, 0.99908918, 0.04240001])
ELEVATION_AXIS = np.array([-0.99826136, 0.0023, 0.05889802])
UPPER_ARM_REFERENCE = np.array([0.0, -1.0, 0.0])
ELBOW_AXIS = np.array([0.0, 0.0, 1.0])
# [elv_angle, shoulder_elv, shoulder_rot, elbow_flexion]
LOWER_LIMITS = np.array([-1.65806279, 0.0, -1.57079633, 0.0])
UPPER_LIMITS = np.array([2.26892803, np.pi, 2.09439510, 2.53073])

# Order of the landmarks in topics.keypoints_2d (mediapipe_kp.KEYPOINTS).
INPUT_KEYPOINTS = ('right_wrist', 'right_elbow', 'right_shoulder')
# Order of the points in topics.arm_keypoints.
KEYPOINT_ORDER = ('right_shoulder', 'right_elbow', 'right_wrist')
# Fields of topics.arm_keypoints_timing, one record per keypoint set: *_stamp
# are sensor stamps, *_wall Unix seconds on the system clock.
TIMING_FIELDS = (
    'cloud_stamp',                # = the arm_keypoints stamp
    'image_stamp',                # the camera image the landmarks came from
    'landmarks_received_wall',
    'cloud_received_wall',
    'keypoints_published_wall',
)

MEASURED_COLOR = (1.0, 0.65, 0.0)
REBUILT_COLOR = (0.0, 1.0, 0.0)
ARM_LINE_COLOR = (0.9, 0.9, 0.9)
ARM_LINE_WIDTH = 0.012


def normalize(vector: np.ndarray) -> np.ndarray:
    """Return the unit vector; raise on near-zero input."""
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        raise ValueError('Cannot normalize a near-zero vector.')
    return vector / norm


def safe_normalize(vector: np.ndarray) -> Optional[np.ndarray]:
    """Return the unit vector, or None when the input is too small."""
    norm = np.linalg.norm(vector)
    return None if norm < 1e-9 else vector / norm


def stamp_seconds(stamp) -> float:
    """Return a builtin_interfaces/Time in seconds; NaN for None."""
    return math.nan if stamp is None else stamp.sec + stamp.nanosec * 1e-9


def axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    return R.from_rotvec(angle * normalize(np.asarray(axis, dtype=float))).as_matrix()


def shoulder_rotation(elv_angle: float, shoulder_elv: float, shoulder_rot: float) -> np.ndarray:
    """Return the rotation of the upper arm in the shoulder frame."""
    return (
        axis_angle_rotation(SHOULDER_AXIS, elv_angle)
        @ axis_angle_rotation(ELEVATION_AXIS, shoulder_elv)
        @ axis_angle_rotation(SHOULDER_AXIS, -elv_angle)
        @ axis_angle_rotation(SHOULDER_AXIS, shoulder_rot)
    )


def upper_arm_direction(elv_angle: float, shoulder_elv: float, shoulder_rot: float) -> np.ndarray:
    """Return the unit humerus direction in the shoulder frame."""
    rotation = shoulder_rotation(elv_angle, shoulder_elv, shoulder_rot)
    return normalize(rotation @ UPPER_ARM_REFERENCE)


def forearm_direction(elv_angle: float, shoulder_elv: float, shoulder_rot: float,
                      elbow_flexion: float) -> np.ndarray:
    """Return the unit forearm direction in the shoulder frame."""
    return normalize(
        shoulder_rotation(elv_angle, shoulder_elv, shoulder_rot)
        @ axis_angle_rotation(ELBOW_AXIS, elbow_flexion)
        @ UPPER_ARM_REFERENCE
    )


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.arccos(np.clip(np.dot(normalize(a), normalize(b)), -1.0, 1.0)))


class ShoulderAnchor:
    """
    The broadcast shoulder pose.

    Follows the measured pose for `init_window_sec` from the first sample, then
    holds the per-axis median position and the mean orientation of those
    samples. A pose is (position, quaternion xyzw) in the camera optical frame.
    """

    def __init__(self, init_window_sec: float):
        self.init_window_sec = init_window_sec
        self.frozen = False
        self.pose: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.sample_count = 0
        self._first_stamp_sec: Optional[float] = None
        self._samples = []

    def add_sample(self, position: np.ndarray, quaternion: np.ndarray, stamp_sec: float) -> None:
        """Take one measured shoulder pose; ignored once frozen."""
        if self.frozen:
            return
        self.pose = (position, quaternion)
        if self._first_stamp_sec is None:
            self._first_stamp_sec = stamp_sec
        self._samples.append(self.pose)
        if stamp_sec - self._first_stamp_sec >= self.init_window_sec:
            positions = np.array([position for position, _ in self._samples])
            rotations = R.from_quat([quaternion for _, quaternion in self._samples])
            self.pose = (np.median(positions, axis=0), rotations.mean().as_quat())
            self.sample_count = len(self._samples)
            self.frozen = True
            self._samples = []


class ArmSolverNode(Node):
    """Landmark pixels + organized cloud -> 3D shoulder, elbow, wrist."""

    def __init__(self):
        super().__init__('swivel_solver_node')

        def text(name):
            return required_parameter(self, name, Parameter.Type.STRING)

        def number(name):
            return required_parameter(self, name, Parameter.Type.DOUBLE)

        keypoints_topic = text('topics.keypoints_2d')
        cloud_topic = text('topics.camera_points')
        self.camera_frame = text('frames.camera')
        self.shoulder_frame = text('frames.arm_prefix') + 'RightShoulder'
        self.upper_arm_length = number('subject.upper_arm_length')
        self.forearm_length = number('subject.forearm_length')

        self.create_subscription(PolygonStamped, keypoints_topic, self.keypoints_callback, 1)
        self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 1)
        self.keypoint_publisher = self.create_publisher(
            PoseArray, text('topics.arm_keypoints'), 10)
        self.valid_publisher = self.create_publisher(Bool, text('topics.arm_keypoints_valid'), 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, text('topics.arm_keypoint_markers'), 10)
        self.timing_publisher = self.create_publisher(
            Float64MultiArray, text('topics.arm_keypoints_timing'), 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.required_topics = (keypoints_topic, cloud_topic)
        self.watchdog = TopicWatchdog(self, self.required_topics)

        self.shoulder = ShoulderAnchor(SHOULDER_INIT_WINDOW_SEC)
        self.keypoints = [None] * len(INPUT_KEYPOINTS)
        # Stamp of the image the landmarks came from, and when they arrived.
        self.keypoints_stamp = None
        self.keypoints_received_wall = float('nan')
        self.cloud = None
        self.last_cloud_ns = None
        # Warm start of the solve; only shoulder_rot carries over between frames.
        self.previous_solution = np.zeros(4)

        self.get_logger().info(
            f'Arm solver ready: upper_arm={self.upper_arm_length:.3f} m, '
            f'forearm={self.forearm_length:.3f} m.')

    # =========================================================================
    # Inputs
    # =========================================================================

    def keypoints_callback(self, msg: PolygonStamped) -> None:
        received_wall = time.time()
        self.watchdog.mark(self.required_topics[0])
        points = msg.polygon.points
        if len(points) < len(INPUT_KEYPOINTS):
            self.get_logger().warn(
                f'Incomplete keypoint message: {len(points)} landmarks, '
                f'expected {len(INPUT_KEYPOINTS)}.', throttle_duration_sec=2.0)
            return
        for i, point in enumerate(points[:len(INPUT_KEYPOINTS)]):
            # Pixel column, row; visibility arrives in percent.
            self.keypoints[i] = np.array([point.x, point.y, point.z / 100.0], dtype=float)
        self.keypoints_stamp = msg.header.stamp
        self.keypoints_received_wall = received_wall

    def cloud_callback(self, msg: PointCloud2) -> None:
        """One frame: read the points, check them, solve, publish."""
        received_wall = time.time()
        self.watchdog.mark(self.required_topics[1])
        if not self.accept_cloud(msg.header.stamp):
            return
        if self.shoulder.frozen:
            self.broadcast_shoulder_tf(self.shoulder.pose, msg.header.stamp)
        if any(kp is None for kp in self.keypoints):
            self.get_logger().warn('Keypoints not available yet.', throttle_duration_sec=2.0)
            self.publish_valid(False)
            return

        self.cloud = msg
        points = {}
        for name, keypoint in zip(INPUT_KEYPOINTS, self.keypoints):
            point = self.point_at_landmark(name, keypoint)
            if point is not None:
                points[name] = point

        # The shoulder frame goes out even on a frame the arm solve rejects.
        if 'right_shoulder' in points and not self.shoulder.frozen:
            stamp = msg.header.stamp
            self.shoulder.add_sample(points['right_shoulder'], SHOULDER_FRAME_QUATERNION,
                                     stamp_seconds(stamp))
            if self.shoulder.frozen:
                self.get_logger().info(
                    f'Shoulder frame fixed at {self.shoulder.pose[0].round(3).tolist()} m in '
                    f'{self.camera_frame} (median of {self.shoulder.sample_count} samples).')
            self.broadcast_shoulder_tf(self.shoulder.pose, stamp)

        if not all(name in points for name in KEYPOINT_ORDER):
            self.get_logger().warn(
                'No usable depth for one of the arm landmarks.', throttle_duration_sec=2.0)
            self.publish_valid(False)
            return

        shoulder, wrist = points['right_shoulder'], points['right_wrist']
        rejection = self.implausible_bone(shoulder, points['right_elbow'], wrist)
        if rejection is not None:
            self.get_logger().warn(f'Rejecting frame: {rejection}.', throttle_duration_sec=2.0)
            self.publish_valid(False)
            return

        elbow = self.solve_arm(shoulder, points['right_elbow'], wrist)
        if elbow is None:
            self.publish_valid(False)
            return

        self.publish_keypoints(msg.header.stamp, shoulder, elbow, wrist)
        published_wall = time.time()
        self.publish_valid(True)
        self.publish_arm_markers(msg.header.stamp, shoulder, elbow, wrist, points['right_elbow'])
        self.publish_timing(msg.header.stamp, received_wall, published_wall)

    def accept_cloud(self, stamp) -> bool:
        """Rate-limit on the sensor clock, so the playback speed does not change the rate."""
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        if self.last_cloud_ns is None:
            self.last_cloud_ns = stamp_ns
            return True
        if stamp_ns - self.last_cloud_ns < int(1e9 / MAX_PROCESSING_RATE_HZ):
            return False
        self.last_cloud_ns = stamp_ns
        return True

    def point_at_landmark(self, name: str, keypoint: np.ndarray) -> Optional[np.ndarray]:
        """Mean of the valid depth samples around one landmark, moved to the joint centre."""
        if float(keypoint[2]) < MIN_CONFIDENCE:
            return None
        samples = self.samples_at_pixel(int(round(keypoint[0])), int(round(keypoint[1])))
        if len(samples) < MIN_VALID_SAMPLES:
            self.get_logger().warn(
                f'Only {len(samples)} valid depth samples at {name}; skipping.',
                throttle_duration_sec=2.0)
            return None
        point = np.mean(samples, axis=0)
        if name == 'right_shoulder':
            point = point + SHOULDER_OFFSET
        elif name in DEPTH_OFFSETS:
            # The optical frame's origin is the camera, so the point is the viewing ray.
            ray = safe_normalize(point)
            if ray is not None:
                point = point + DEPTH_OFFSETS[name] * ray
        return point

    def samples_at_pixel(self, u: int, v: int) -> np.ndarray:
        """Return the valid (x, y, z) in the pixel patch around (u, v) of the cloud."""
        width, height = self.cloud.width, self.cloud.height
        radius = SAMPLE_RADIUS_PX
        uvs = [
            su + sv * width
            for sv in range(v - radius, v + radius + 1)
            for su in range(u - radius, u + radius + 1)
            if 0 <= su < width and 0 <= sv < height
        ]
        if not uvs:
            return np.empty((0, 3))
        # Not skip_nans: that drops rows before `uvs` is applied. The driver marks
        # invalid points with zeros; the filter below rejects those and NaNs.
        points = pc2.read_points(self.cloud, field_names=('x', 'y', 'z'), uvs=uvs)
        return np.array(
            [[p[0], p[1], p[2]] for p in points
             if np.isfinite(p[0]) and np.isfinite(p[1]) and np.isfinite(p[2])
             and not (p[0] == 0.0 and p[1] == 0.0 and p[2] == 0.0)],
            dtype=float)

    # =========================================================================
    # Solve
    # =========================================================================

    def implausible_bone(self, shoulder, elbow, wrist) -> Optional[str]:
        """Describe the first bone whose measured length is anatomically wrong, if any."""
        for label, measured, nominal in (
            ('upper arm', np.linalg.norm(elbow - shoulder), self.upper_arm_length),
            ('forearm', np.linalg.norm(wrist - elbow), self.forearm_length),
        ):
            low, high = SEGMENT_LENGTH_RATIO[0] * nominal, SEGMENT_LENGTH_RATIO[1] * nominal
            if not low <= measured <= high:
                return (f'{label} measures {measured:.3f} m, outside the '
                        f'[{low:.3f}, {high:.3f}] m expected for a {nominal:.3f} m segment')
        return None

    def solve_arm(self, shoulder, raw_elbow, wrist) -> Optional[np.ndarray]:
        """Return the rebuilt elbow in the camera frame, or None."""
        to_shoulder = CAMERA_TO_SHOULDER_ROTATION
        initial = self.solve_all_angles(
            to_shoulder @ (raw_elbow - shoulder), to_shoulder @ (wrist - raw_elbow))
        if initial is None:
            return None

        elv_angle = float(np.clip(initial[0] + Q0_OFFSET, LOWER_LIMITS[0], UPPER_LIMITS[0]))
        shoulder_elv = float(np.clip(initial[1] + Q1_OFFSET, LOWER_LIMITS[1], UPPER_LIMITS[1]))

        # The elbow at the subject's upper-arm length, on the corrected humerus direction.
        upper_direction = upper_arm_direction(elv_angle, shoulder_elv, float(initial[2]))
        elbow = shoulder + SHOULDER_TO_CAMERA_ROTATION @ (self.upper_arm_length * upper_direction)

        measured_forearm = safe_normalize(to_shoulder @ (wrist - elbow))
        if measured_forearm is None:
            self.get_logger().warn(
                'Rebuilt elbow coincides with the measured wrist; skipping frame.',
                throttle_duration_sec=2.0)
            return None
        # shoulder_rot carries into the next frame's guess: keep the re-solved value.
        self.previous_solution[2] = self.solve_shoulder_rotation(
            elv_angle, shoulder_elv, initial[2:], upper_direction, measured_forearm)
        return elbow

    def solve_all_angles(self, upper_arm, forearm) -> Optional[np.ndarray]:
        """Solve q0..q3 from the two bone directions (the forearm fixes shoulder_rot)."""
        upper = safe_normalize(upper_arm)
        measured_forearm = safe_normalize(forearm)
        if upper is None or measured_forearm is None:
            self.get_logger().error('Degenerate bone vectors; skipping frame.')
            return None

        flexion_guess = angle_between(upper, measured_forearm)
        guess = self.previous_solution.copy()
        guess[0] = np.sign(upper[0]) * np.arctan2(abs(upper[0]), abs(upper[2]))
        guess[1] = (np.arctan2(upper[2], -upper[1]) if upper[1] < 0.0
                    else np.pi - np.arctan2(upper[2], upper[1]))
        guess[3] = flexion_guess

        def residual(q):
            return np.concatenate((
                3.0 * (upper_arm_direction(q[0], q[1], q[2]) - upper),
                forearm_direction(q[0], q[1], q[2], q[3]) - measured_forearm,
                [0.15 * (q[3] - flexion_guess)],
            ))

        result = least_squares(
            residual, np.clip(guess, LOWER_LIMITS, UPPER_LIMITS),
            bounds=(LOWER_LIMITS, UPPER_LIMITS), xtol=1e-5, ftol=1e-5, gtol=1e-5, max_nfev=60)
        if not result.success:
            self.get_logger().warn(
                f'Shoulder solve did not converge: {result.message}', throttle_duration_sec=2.0)
        self.previous_solution = result.x
        return result.x

    def solve_shoulder_rotation(self, elv_angle, shoulder_elv, guess,
                                upper_direction, measured_forearm) -> float:
        """Re-solve shoulder_rot and elbow flexion with the humerus fixed; return shoulder_rot."""
        flexion_guess = angle_between(upper_direction, measured_forearm)
        bounds = (LOWER_LIMITS[2:], UPPER_LIMITS[2:])

        def residual(q):
            return np.concatenate((
                forearm_direction(elv_angle, shoulder_elv, q[0], q[1]) - measured_forearm,
                [0.15 * (q[1] - flexion_guess)],
            ))

        result = least_squares(
            residual, np.clip(guess, *bounds), bounds=bounds,
            xtol=1e-5, ftol=1e-5, gtol=1e-5, max_nfev=60)
        if not result.success:
            self.get_logger().warn(
                f'Forearm re-solve did not converge: {result.message}', throttle_duration_sec=2.0)
        return float(result.x[0])

    # =========================================================================
    # Outputs
    # =========================================================================

    def publish_keypoints(self, stamp, shoulder, elbow, wrist) -> None:
        """Publish the three joint centres in KEYPOINT_ORDER."""
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        for position in (shoulder, elbow, wrist):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.keypoint_publisher.publish(msg)

    def publish_timing(self, cloud_stamp, cloud_received_wall: float,
                       published_wall: float) -> None:
        """Publish one TIMING_FIELDS record for the keypoints just published."""
        values = (stamp_seconds(cloud_stamp), stamp_seconds(self.keypoints_stamp),
                  self.keypoints_received_wall, cloud_received_wall, published_wall)
        self.timing_publisher.publish(Float64MultiArray(data=[float(v) for v in values]))

    def publish_valid(self, valid: bool) -> None:
        self.valid_publisher.publish(Bool(data=bool(valid)))

    def publish_arm_markers(self, stamp, shoulder, elbow, wrist, measured_elbow) -> None:
        """Spheres at the published points and the measured elbow, and the arm line."""
        markers = MarkerArray()
        for marker_id, (name, position, color) in (
            (0, ('right_shoulder', shoulder, MEASURED_COLOR)),
            (1, ('right_elbow', elbow, REBUILT_COLOR)),
            (2, ('right_wrist', wrist, MEASURED_COLOR)),
            (4, ('right_elbow_measured', measured_elbow, MEASURED_COLOR)),
        ):
            marker = self.new_marker(marker_id, stamp, Marker.SPHERE)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                float(v) for v in position)
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * MARKER_RADIUS
            marker.color.r, marker.color.g, marker.color.b = color
            marker.text = name
            markers.markers.append(marker)

        line = self.new_marker(3, stamp, Marker.LINE_STRIP)
        line.scale.x = ARM_LINE_WIDTH
        line.color.r, line.color.g, line.color.b = ARM_LINE_COLOR
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                       for p in (shoulder, elbow, wrist)]
        markers.markers.append(line)
        self.marker_publisher.publish(markers)

    def new_marker(self, marker_id: int, stamp, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame
        marker.ns = 'arm_joints'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=int(MARKER_LIFETIME_SEC),
                                   nanosec=int((MARKER_LIFETIME_SEC % 1.0) * 1e9))
        return marker

    def broadcast_shoulder_tf(self, pose, stamp) -> None:
        """Broadcast camera -> shoulder frame."""
        position, quaternion = pose
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.shoulder_frame
        (transform.transform.translation.x,
         transform.transform.translation.y,
         transform.transform.translation.z) = (float(v) for v in position)
        (transform.transform.rotation.x,
         transform.transform.rotation.y,
         transform.transform.rotation.z,
         transform.transform.rotation.w) = (float(v) for v in quaternion)
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    run_node(ArmSolverNode, args=args)


if __name__ == '__main__':
    main()
