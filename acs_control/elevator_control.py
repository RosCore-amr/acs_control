#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta, timezone

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.srv import AddTwoInts
from functools import partial
from robot_interfaces.msg import MissionTransport, MissionCurrent, Token
from robot_interfaces.srv import GetInformation, CommandApi, CommonRequest
import requests
from enum import Enum


class TaskStatus(Enum):

    CALL_ELEVATOR = 1
    IN_ELEVATOR = 2
    INIT_LOCATION = 3
    OUT_ELEVATOR = 4
    FINISH = 5


class AcsElevator(Node):

    def __init__(self):
        super().__init__("elevator_acs_control")
        # self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 1  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )
        self.srv_process_excute_mission = self.create_service(
            CommonRequest, "processing_elevator", self.excute_process_srv
        )

        self.cli_get2system = self.create_client(GetInformation, "get_from_system")

        self.publisher_elevator_status = self.create_publisher(
            String, "elevator_status", 10
        )
        self.robot_work_status = self.create_subscription(
            String, "robot_work_ability", self.robot_ability_work_callback, 10
        )
        self.sub_elevator_status = self.create_subscription(
            String, "elevator_status", self.elevator_status_callback, 10
        )

        self.publisher_command_mosbus = self.create_publisher(
            String, "acs_communication", 10
        )

        self.cli_collison_acs = self.create_client(CommonRequest, "collison_acs")

        self._list_elevator_status = []
        self.dict_elevator_work = {}
        self.elevator_position_robot = {}
        self.dict_robot_work = {}
        self.robot_work_elevator = []
        self.sever_work = True
        # self.subscription_sever_workl_ = self.create_subscription(
        #     Token, "token", self.sever_callback, 10
        # )
        # self.take_information_elevator()

    def take_information_elevator(self):
        # if self.sever_work:
        _elevator_status = self.get_inform_system_client("get_elevator_all")
        # self.get_logger().info('_elevator_status: "%s"' % _elevator_status)
        self._list_elevator_status = eval(_elevator_status.msg_response)
        return self._list_elevator_status

    def sever_callback(self, msg):
        self.sever_work = msg.server_work
        # self.take_information_elevator()
        # self.get_logger().info(
        #     '_list_elevator_status: "%s"' % self._list_elevator_status
        # )

    def excute_process_srv(self, request, response):
        # request_db = {
        #     "excute_code": request.excute_code,
        #     "mission_excute": request.value,
        # }
        # response_api = self.patch_data_to_database(request.url, request_db)
        _request = eval(request.msg_request)
        _response = self.elevator_processing(_request)
        # self.get_logger().info('_request: "%s"' % _request)
        response.msg_response = str(_response)
        return response

    def check_map_available(self, map_check):
        _available_map = ["pickup_locations", "return_locations"]
        if map_check not in _available_map:
            return {"code": 0, "msg": "map is not available"}

    def elevator_processing(self, _request):
        # if _request[]
        # for i
        # self.get_logger().info('before_elevator_work: "%s"' % self.dict_elevator_work)
        _check_robot_and_elevator = self.comfirm_robot_activity(_request)
        if not _check_robot_and_elevator["elevator_reserve"]:
            if _request["step"] != TaskStatus.CALL_ELEVATOR.value:
                return {
                    "code": 0,
                    "elevator_code": "error",
                    "msg": "robot not on list ",
                }
        else:
            _position_current = self.elevator_position_robot[
                _check_robot_and_elevator["elevator_code"]
            ]

        if _request["step"] == TaskStatus.CALL_ELEVATOR.value:
            if _check_robot_and_elevator["elevator_reserve"]:
                return {
                    "code": 0,
                    "elevator_code": "error",
                    "msg": "robot have elevator not called ",
                }
            reserve_elevator = self.find_elevator_for_robot(_request)
            # self.get_logger().info(
            #     'elevator_reserve: "%s"' % reserve_elevator["elevator_code"]
            # )

            if not reserve_elevator["code"]:
                return reserve_elevator
            _position_wait = self.elevator_position_robot[
                reserve_elevator["elevator_code"]
            ][_request["current_map"]]

            _position_robot_need_tobe = _position_wait["possition_out"]
            return {"code": 1, "position": _position_robot_need_tobe}
        elif _request["step"] == TaskStatus.IN_ELEVATOR.value:

            # _position_current = self.elevator_position_robot[
            #     _check_robot_and_elevator["elevator_code"]
            # ]

            _call_elevator = self.call_elevator(
                _position_current[_request["current_map"]],
                _position_current[_request["destination_map"]],
                _position_current["acs_id"],
                _request["robot_code"],
            )

            _position_robot_need_tobe = _position_current[_request["current_map"]][
                "position_in"
            ]
            return {"code": 1, "position": _position_robot_need_tobe}

        elif _request["step"] == TaskStatus.INIT_LOCATION.value:

            self.call_elevator_run(
                _position_current["acs_id"],
                _position_current[_request["destination_map"]],
                _request["robot_code"],
            )

            _position_robot_need_tobe = _position_current[_request["destination_map"]][
                "position_in"
            ]
            return {"code": 1, "position": _position_robot_need_tobe}

        elif _request["step"] == TaskStatus.OUT_ELEVATOR.value:

            _position_robot_need_tobe = _position_current[_request["destination_map"]][
                "possition_out"
            ]
            return {"code": 1, "position": _position_robot_need_tobe}

        elif _request["step"] == TaskStatus.FINISH.value:

            self.exit_elevator(
                _position_current["acs_id"],
                _position_current[_request["destination_map"]],
                _request["robot_code"],
            )

            _finish_activities_elevator = {
                _check_robot_and_elevator["elevator_code"]: {
                    "reserve_elevator": {
                        "robot_work": None,
                        "elevator_operating": False,
                        "step": 0,
                    }
                }
            }
            self.dict_elevator_work.update(_finish_activities_elevator)
            # self.get_logger().error('elevator_reserve: "%s"' % self.dict_elevator_work)

            return {"code": 1, "msg": "finish activities with elevator"}
        return str({"test": "success"})

    def find_robot_name(self, ip_machine, _type_name):
        # self.dict_robot_work
        if ip_machine in self.dict_robot_work.keys():
            return self.dict_robot_work[ip_machine][_type_name]
        return ip_machine

    def robot_ability_work_callback(self, msg):
        self.dict_robot_work = eval(msg.data)

    def elevator_status_callback(self, msg):
        pass

    def call_elevator(self, _current_map, _destinaton_map, _acs_code, _robot_code):

        if int(_current_map["floor"]) < int(_destinaton_map["floor"]):
            comm = (
                "CALLU0"
                + str(_acs_code)
                + str(0)
                + str(_current_map["floor"])
                + str(260)
            )
        else:
            comm = (
                "CALLD0"
                + str(_acs_code)
                + str(0)
                + str(_current_map["floor"])
                + str(260)
            )
        _dict_request_communication = str({"commnad": comm, "agv_id": _robot_code})
        self.get_logger().info(
            '_dict_request_communication: "%s"' % _dict_request_communication
        )
        self.pub_communication_with_acs(_dict_request_communication)
        # response_acs = self.commucation_collision_acs_client(
        #     _dict_request_communication
        # )
        # self.get_logger().info('response_acs: "%s"' % response_acs)

        # self.get_logger().info('response_acs: "%s"' % response_acs.msg_response)

    def call_elevator_run(self, _acs_code, _destinaton_map, _robot_code):

        start_floor = _destinaton_map["floor"]
        # elevator_running(self, agv_id, ev_id, f_b, start_floor):
        comm = "CALL00" + str(_acs_code) + str(0) + str(start_floor)
        _dict_request_communication = str({"commnad": comm, "agv_id": _robot_code})
        self.pub_communication_with_acs(_dict_request_communication)

    def exit_elevator(self, _acs_code, _destinaton_map, _robot_code):

        start_floor = _destinaton_map["floor"]
        comm = "OUTFN0" + str(_acs_code) + str(0) + str(start_floor)
        _dict_request_communication = str({"commnad": comm, "agv_id": _robot_code})
        self.pub_communication_with_acs(_dict_request_communication)

    def pub_communication_with_acs(self, _data_request):
        msg = String()
        msg.data = str(_data_request)
        self.publisher_command_mosbus.publish(msg)

    def comfirm_robot_activity(self, _request):
        for elevator_name, value_elevator in self.dict_elevator_work.items():
            if (
                value_elevator["reserve_elevator"]["robot_work"]
                == _request["robot_code"]
            ):
                return {
                    "elevator_reserve": True,
                    "elevator_code": elevator_name,
                    "step": value_elevator["reserve_elevator"]["step"],
                }
        return {"elevator_reserve": False, "elevator_code": None}

    def get_inform_system_client(self, _url):
        req = GetInformation.Request()
        while not self.cli_get2system.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = _url
        future = self.cli_get2system.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
        return None

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

    def publish_elevator_status(self):
        # temperature = random.randint(20, 30)
        msg = String()
        msg.data = str(self.dict_elevator_work)
        self.publisher_elevator_status.publish(msg)

    def find_elevator_for_robot(self, _request_code):

        # self.get_logger().info(f"value: { self.dict_elevator_work}")
        for elevator_name, value_elevator in self.dict_elevator_work.items():
            # self.get_logger().info(value_elevator)
            # self.get_logger().info(f"value_elevator: {value_elevator}")
            if not value_elevator["reserve_elevator"]["elevator_operating"]:
                dict_reserve = {
                    elevator_name: {
                        "reserve_elevator": {
                            "robot_work": _request_code["robot_code"],
                            "elevator_operating": True,
                            "step": 1,
                        },
                        # "acs_id": value_elevator["acs_id"],
                    }
                }
                self.dict_elevator_work.update(dict_reserve)
                return {"code": 1, "elevator_code": elevator_name}
        return {"code": 0, "elevator_code": "error"}

    def consider_ability_elevator(self, elevator):
        # self.get_logger().info(f"elevator: {elevator}")
        self.dict_elevator_work.update(elevator)
        # self.get_logger().info(f"dict_elevator_work: {self.dict_elevator_work}")

    def initial_ability_elevator(self):
        for i in range(len(self._list_elevator_status)):
            _dict_elevator_work = {
                self._list_elevator_status[i]["elevator_name"]: {
                    # "current_status": {
                    #     "current_map": self._list_elevator_status[i]["elevator_status"][
                    #         "map_position"
                    #     ],
                    #     "door_open": self._list_elevator_status[i]["elevator_status"][
                    #         "door_open"
                    #     ],
                    # },
                    # "acs_id": self._list_elevator_status[i]["acs_id"],
                    "reserve_elevator": {
                        "elevator_operating": False,
                        "robot_work": None,
                        "step": 0,
                    },
                    # "robot_position": self._list_elevator_status[i]["robot_position"],
                }
            }
            _dict_robot_elevator_position = {
                self._list_elevator_status[i][
                    "elevator_name"
                ]: self._list_elevator_status[i]["robot_position"],
                # "acs": self._list_elevator_status[i]["acs_id"],
            }
            self.elevator_position_robot.update(_dict_robot_elevator_position)
            self.consider_ability_elevator(_dict_elevator_work)
            # self.get_logger().info('_dict_elevator_work: "%s"' % _dict_elevator_work)

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
        if not len(self._list_elevator_status):
            self.take_information_elevator()
            reservation_elevator = {}
            self.initial_ability_elevator()
        self.publish_elevator_status()

        _dict_request_communication = str({"asdas": "Asdasd"})
        # response_acs = self.commucation_collision_acs_client(
        #     _dict_request_communication
        # )
        # self.get_logger().error(" pub lish auto")
        # self.find_elevator_for_robot("robot_1")
        # self.get_logger().info('response_acs: "%s"' % response_acs)

        # self.elevator_processing()


def main(args=None):
    rclpy.init(args=args)
    elevator_acs_control = AcsElevator()
    executor = MultiThreadedExecutor()
    executor.add_node(elevator_acs_control)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
