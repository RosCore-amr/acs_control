import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta, timezone

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.srv import AddTwoInts
from functools import partial
from robot_interfaces.msg import MissionTransport, MissionCurrent
from robot_interfaces.srv import GetInformation, CommandApi
import requests


class AcsElevator(Node):

    def __init__(self):
        super().__init__("elevator_acs_control")
        # self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 0.5  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )

    def sent_patch(self, _url, _request_body):

        try:
            res = requests.patch(
                _url,
                # headers=self.__token_db,
                json=_request_body,
                timeout=3,
            )
            response = res.json()
            if response["metaData"]:
                return None
            return True
        except Exception as e:
            return None

    def main_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    elevator_acs_control = AcsElevator()
    executor = MultiThreadedExecutor()
    executor.add_node(elevator_acs_control)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
