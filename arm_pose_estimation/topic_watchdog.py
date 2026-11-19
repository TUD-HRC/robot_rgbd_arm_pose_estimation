"""
Log an error while a required input topic is silent, naming the topic.

The check timer runs on a steady clock, so it also reports when /clock never
arrives (use_sim_time with no recording playing).
"""

from typing import Dict, Iterable, List, Optional

from rclpy.clock import Clock, ClockType
from rclpy.node import Node


REPORT_PERIOD_SEC = 5.0
SILENCE_TIMEOUT_SEC = 5.0
# A topic with a publisher may take this long to deliver its first message.
FIRST_MESSAGE_GRACE_SEC = 10.0


class TopicWatchdog:
    """Report silent input topics; call mark(topic) on every message."""

    def __init__(self, node: Node, topics: Iterable[str]) -> None:
        """Watch `topics` on `node`."""
        self._node = node
        self._last_seen: Dict[str, Optional[float]] = {topic: None for topic in topics}
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._start = self._now()
        self._timer = node.create_timer(REPORT_PERIOD_SEC, self._check, clock=self._clock)

    def _now(self) -> float:
        return self._clock.now().nanoseconds * 1e-9

    def mark(self, topic: str) -> None:
        """Record that a message arrived on `topic`."""
        if topic not in self._last_seen:
            return
        if self._last_seen[topic] is None:
            self._node.get_logger().info(f'Receiving {topic}.')
        self._last_seen[topic] = self._now()

    def _check(self) -> None:
        now = self._now()
        problems: List[str] = []
        for topic, last_seen in self._last_seen.items():
            if last_seen is None:
                publishers = self._node.count_publishers(topic)
                if publishers == 0:
                    problems.append(f'{topic}: no publisher')
                elif now - self._start > FIRST_MESSAGE_GRACE_SEC:
                    problems.append(f'{topic}: {publishers} publisher(s) but no message received')
            elif now - last_seen > SILENCE_TIMEOUT_SEC:
                problems.append(f'{topic}: silent for {now - last_seen:.1f} s')
        if problems:
            self._node.get_logger().error(
                'Required input topic(s) unavailable:\n  ' + '\n  '.join(problems))
