"""
2D arm landmarks from the RGB image with MediaPipe Pose.

For every camera image, publishes the right wrist, elbow and shoulder
(KEYPOINTS order) as one geometry_msgs/PolygonStamped, stamped with the image:
x, y are the pixel column and row, z the visibility in percent. Nothing is
published for an image without a detected person. The image with the
skeleton drawn on it goes out on topics.pose_annotated.

Needs the legacy `mp.solutions` API (mediapipe 0.10.21).
"""

from arm_pose_estimation.node_main import required_parameter, run_node
from arm_pose_estimation.topic_watchdog import TopicWatchdog
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PolygonStamped
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


# MediaPipe Pose landmark index of each published keypoint, in output order.
KEYPOINTS = (
    ('right_wrist', 16),
    ('right_elbow', 14),
    ('right_shoulder', 12),
)
MODEL_COMPLEXITY = 1
MIN_DETECTION_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE = 0.5


class MediaPipePoseNode(Node):
    """Publish the arm landmarks of each camera image."""

    def __init__(self):
        super().__init__('mediapipe_pose_node')
        import mediapipe as mp

        def topic(name):
            return required_parameter(self, f'topics.{name}', Parameter.Type.STRING)

        self.rgb_topic = topic('camera_rgb')
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=MODEL_COMPLEXITY,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=MIN_TRACKING_CONFIDENCE)
        self.bridge = CvBridge()

        self.keypoint_publisher = self.create_publisher(PolygonStamped, topic('keypoints_2d'), 10)
        self.annotated_publisher = self.create_publisher(Image, topic('pose_annotated'), 10)
        self.create_subscription(Image, self.rgb_topic, self.on_image, 10)
        self.watchdog = TopicWatchdog(self, [self.rgb_topic])
        self.get_logger().info(f'MediaPipe Pose running on {self.rgb_topic}.')

    def on_image(self, msg: Image) -> None:
        self.watchdog.mark(self.rgb_topic)
        image_rgb = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2RGB)
        result = self.pose.process(image_rgb)
        if not result.pose_landmarks:
            self.get_logger().warn('No pose landmarks detected', throttle_duration_sec=2.0)
            return

        height, width = image_rgb.shape[:2]
        landmarks = result.pose_landmarks.landmark
        keypoints = PolygonStamped()
        keypoints.header = msg.header
        for _, index in KEYPOINTS:
            landmark = landmarks[index]
            keypoints.polygon.points.append(Point32(
                x=float(int(np.clip(landmark.x * width, 0, width - 1))),
                y=float(int(np.clip(landmark.y * height, 0, height - 1))),
                z=float(int(np.clip(landmark.visibility, 0.0, 1.0) * 100))))
        self.keypoint_publisher.publish(keypoints)

        annotated = image_rgb.copy()
        self.mp_drawing.draw_landmarks(
            annotated, result.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        annotated_msg.header = msg.header
        self.annotated_publisher.publish(annotated_msg)


def main(args=None):
    run_node(MediaPipePoseNode, args=args)


if __name__ == '__main__':
    main()
