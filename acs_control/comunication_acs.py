import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import MissionTransport
from robot_interfaces.srv import CommandApi, CommonRequest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.srv import AddTwoInts
from functools import partial
from ament_index_python.packages import get_package_share_directory

# from amd_sevtsv import ServerControl
import os
import yaml
import json
import socket
import random
import select
import signal

STX = 0x02
ETX = 0x03


class ACSControlSocket(Node):

    def __init__(self):
        super().__init__("acs_communication_socket")
        timer_period = 1  # seconds

        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )
        self.srv_collision_acs = self.create_service(
            CommonRequest, "collison_acs", self.collision_acs_srv
        )
        self.initial_protocol()
        self._host = "107.113.5.242"
        self._port = 8082
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def initial_protocol(self):
        config_path_current_robot = os.path.join(
            get_package_share_directory("acs_control"),
            "config",
            "socket_protocal.json",
        )
        data_socket = self.load_config(config_path_current_robot)
        self._host = data_socket["host_acs"]
        self._port = data_socket["port_acs"]
        if self.connect_ethernet():
            self.get_logger().info(f"data_socket: {data_socket}")
        else:
            self.get_logger().info(f"error: {self._host}")

    def connect_ethernet(self):
        self.ethernet_protocal = True
        return True
        try:
            # socket.setdefaulttimeout(5)
            self.ethernet_protocal = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = (self._host, self._port)
            self.ethernet_protocal.connect(server_address)
            # self._client, self._address = self.ethernet_protocal.accept()
            self.ethernet_protocal.settimeout(2)
            # Ethernet_protocal.bind(server_address)
            return True
        except OSError as error:
            self.get_logger().info(f"error: {self._host}")
            return False

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

    def collision_acs_srv(self, request, response):

        self.get_logger().info(f"error: {request.msg_request}")
        response.msg_response = "minh dep trai"
        return response

    def create_frame(self, index_type, ap, msg, agv_id, command):

        if len(command) < 12:
            emp_buf = [0 for _ in range(12 - len(command))]
            command = command + "".join(map(str, emp_buf))

        elif len(command) > 12:
            # logging.info("Command Error")
            pass
        else:
            command = command
        if int(agv_id) < 10:
            raw_array = [index_type, ap, msg, 0, agv_id, command]
        else:
            raw_array = [index_type, ap, msg, agv_id, command]

        list_to_str = "".join(map(str, raw_array))

        raw_msg = bytearray(list_to_str, "ascii")

        raw_msg.insert(0, STX)
        check_sum = 0x00
        for item in raw_msg:
            check_sum += int(item)
        check_sum = check_sum - 2
        check_sum_arr = "".join(map(str, hex(check_sum)))
        check_sum = bytes(check_sum_arr[len(check_sum_arr) - 1].upper(), "utf-8")

        raw_msg += check_sum
        raw_msg.append(ETX)
        return raw_msg

    def listen_rev(self):

        ready = select.select([self.ethernet_protocal], [], [], 5)
        if ready[0]:
            data_all = self.ethernet_protocal.recv(20)
            # data_all = self.ethernet_protocal.recv(4096)
            if (data_all[0] != STX) or (data_all[19] != ETX):
                # logging.warning("Pack head error")
                pass
            else:
                check_sum = 0x00
                for item in range(18):
                    check_sum += int(data_all[item])
                check_sum = check_sum - 2
                check_sum_arr = "".join(map(str, hex(check_sum)))
                check_sum = bytes(
                    check_sum_arr[len(check_sum_arr) - 1].upper(), "utf-8"
                )
                if int.from_bytes(check_sum, "little", signed=False) != data_all[18]:
                    # logging.info("Checksum Error")
                    pass
                else:
                    return {
                        "index": chr(data_all[1]),
                        "ap": int(chr(data_all[2])),
                        "msg": int(chr(data_all[3])),
                        "agv_id": int(data_all[4:6].decode("utf-8")),
                        "command": data_all[6:18].decode("utf-8"),
                    }
        return False

    def main_loop(self):

        request_body = {
            "location_code": "zone9",
            "map_code": "return_locations",
            "excute_code": "transport_goods",
        }
        # self.get_logger().info(f"Result: {request_body}")


def main(args=None):
    rclpy.init(args=args)

    acs_communication_socket = ACSControlSocket()
    executor = MultiThreadedExecutor()
    executor.add_node(acs_communication_socket)
    executor.spin()
    acs_communication_socket.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
