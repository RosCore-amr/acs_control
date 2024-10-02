import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import MissionTransport, CollisionPresent
from robot_interfaces.srv import Collision, CommonRequest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from datetime import datetime, timedelta, timezone
from rclpy.executors import MultiThreadedExecutor

from functools import partial
import os
import yaml
import json
from ament_index_python.packages import get_package_share_directory


class AcsSystemControl(Node):

    def __init__(self):
        super().__init__("acs_sever_control")
        # self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 2  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )

        self.srv_collision = self.create_service(
            Collision, "collision_process", self.collision_srv
        )
        self._collision_publish = self.create_publisher(
            CollisionPresent, "collision_data", 3
        )
        self.cli_collison_acs = self.create_client(CommonRequest, "collison_acs")

        self.element_collision_current = {}
        self.list_collision = []
        self.dict_collision_point = {}

        self.initial_system()

    def initial_system(self):
        _collision_factory = os.path.join(
            get_package_share_directory("acs_control"),
            "config",
            "factory_collisions.json",
        )
        self.factory_collision_requirment = self.load_config(_collision_factory)

    def load_config(self, path):
        config = {}
        with open(path, "r") as stream:
            try:
                d = yaml.load(stream, Loader=yaml.FullLoader)
                for key, val in d.items():
                    config[key] = val
                return config
            except yaml.YAMLError as e:
                print(e)

    def wrap_collision_factory(self, _robot, _possition_collision_current):

        if _possition_collision_current not in self.factory_collision_requirment.keys():
            return True
        else:
            return False

        for (
            collision_point_robot,
            collision_factory,
        ) in self.factory_collision_requirment.items():

            if _possition_collision_current == collision_point_robot:
                self.get_logger().info(
                    '_possition_collision_current: "%s"' % _possition_collision_current
                )
                collision_in_tag = {
                    "robot_code": _robot,
                    "command": _possition_collision_current,
                }
                communication_acs = self.commucation_collision_acs_client(
                    collision_in_tag
                )
                return communication_acs

    def gumshoe_collision(self, robot_current, possition_collision_current):

        wrap_factory = self.wrap_collision_factory(
            robot_current, possition_collision_current
        )
        if not wrap_factory:
            return False

        # self.get_logger().info('test: "%s"' % wrap_factory)
        # self.wrap_collision_factory()
        expired_time = (datetime.now() + timedelta(seconds=30)).strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )
        if not self.list_collision.count(possition_collision_current):
            new_collision = {
                robot_current: {
                    "collision_code": possition_collision_current,
                    "date": expired_time,
                }
            }
            self.dict_collision_point.update(new_collision)
            return True

        for robot, possition_collision in self.dict_collision_point.items():

            if possition_collision_current == possition_collision["collision_code"]:
                if robot_current == robot:
                    possition_collision["date"] = expired_time
                    # time_collision_update = {robot_current: {"date": expired_time}}
                    # self.dict_collision_point.update(time_collision_update)
                    return True

                else:
                    self.get_logger().info('robot_keys_check: "%s"' % "stop robot")
                    return False

    def calc_diff(self, time_start, time_end):

        d_start = datetime.strptime(time_start, "%Y-%m-%d %H:%M:%S.%f")
        d_end = datetime.strptime(time_end, "%Y-%m-%d %H:%M:%S.%f")
        time_result = d_start - d_end
        # self.data_log[2] = d_end - d_start
        if d_start < d_end:
            return False
        return True

    def collision_srv(self, request, response):

        # self.element_collision_current = {
        #     request.robot_code: request.position_collision
        # }
        result_response = self.gumshoe_collision(
            request.robot_code, request.position_collision
        )

        response.result = str({"moving": result_response})
        return response

    def calculate_lifetime_collision(self):
        now_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        for robot, possition_collision in self.dict_collision_point.items():
            time_date_collision = self.calc_diff(now_time, possition_collision["date"])
            if time_date_collision:
                self.dict_collision_point.pop(robot)
                break
            # self.get_logger().info('time_date_collision: "%s"' % time_date_collision)
            # self.get_logger().info(
            #     'date time calculate: "%s"' % possition_collision["date"]
            # )

    def collision_processing(self):
        msg = CollisionPresent()

        _list_collision = list(self.dict_collision_point.values())
        _collision = []
        for i in _list_collision:
            # self.get_logger().info('date time calculate: "%s"' % i["date"])
            _collision.append(i["collision_code"])

        self.list_collision = _collision
        msg.collision_list = _collision
        msg.msg_collision = str(self.msg2json(self.dict_collision_point))
        self._collision_publish.publish(msg)
        self.calculate_lifetime_collision()

        self.get_logger().info(
            'msg_collision: "%s"' % self.msg2json(self.dict_collision_point)
        )

    def commucation_collision_acs_client(self, command_request):

        req = CommonRequest.Request()
        while not self.cli_collison_acs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.msg_request = str(command_request)
        future = self.cli_collison_acs.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
        return None

    def msg2json(self, msg):
        # y = json.load(str(msg))
        return json.dumps(msg, indent=4)

    def main_loop(self):
        self.collision_processing()


def main(args=None):
    rclpy.init(args=args)

    acs_sever_control = AcsSystemControl()
    executor = MultiThreadedExecutor()
    executor.add_node(acs_sever_control)
    executor.spin()
    acs_sever_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
