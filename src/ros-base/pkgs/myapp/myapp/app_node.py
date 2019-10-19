"""
App Node

All implementations related to ROS should be here.
"""


###############################################################################
# IMPORTS
###############################################################################


import atexit
import signal
import sys
from enum import Enum, unique

import rclpy
from rclpy.qos import qos_profile_sensor_data

from myapp_apis.msg import AppCommand
from myapp_apis.msg import AppData
from myapp_apis.srv import GetAppData

from . import app as APP


###############################################################################
# CONSTANTS
###############################################################################


NODE_NAME = "APP"


@unique
class Services(Enum):
    GET_APP_DATA = "get_app_data"


@unique
class Topics(Enum):
    SEND_APP_COMMAND = "send_app_command"


###############################################################################
# CALLBACKS
###############################################################################


def send_app_command_callback(msg: AppCommand):
    print(f"{NODE_NAME} received: x={msg.x},y={msg.y},z={msg.z},action={msg.action}")

    APP.apply_command(
        msg.x,
        msg.y,
        msg.z,
        msg.action)


def get_app_data_callback(request: GetAppData.Request, response: GetAppData.Response):
    #print(repr(request))    # see fields in GetAppData.srv
    #print(repr(response))   # see return type in GetAppData.srv

    user_data = APP.get_data(request.user_id)

    response.data.x = float(user_data.x)
    response.data.y = float(user_data.y)
    response.data.z = float(user_data.z)

    return response


def stop_node(*args):
    rclpy.shutdown()

    # signal handlers should return something True-ish
    return True


###############################################################################
# MAIN
###############################################################################


def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node(NODE_NAME)
    node.create_subscription(
        AppCommand,                     # msg_type
        Topics.SEND_APP_COMMAND.value,  # topic
        send_app_command_callback,      # callback
        qos_profile_sensor_data         # QoS Profile
    )
    node.create_service(
        GetAppData,                     # srv_type
        Services.GET_APP_DATA.value,    # srv_name
        get_app_data_callback           # callback
    )

    print(f"{NODE_NAME} is UP.")

    while rclpy.ok():
        rclpy.spin_once(node)

    print(f"{NODE_NAME} is DOWN.")


if __name__ == '__main__':
    try:
        atexit.register(stop_node)
        signal.signal(signal.SIGABRT, stop_node)
        signal.signal(signal.SIGTERM, stop_node)

        main()
    except:
        print(f"{NODE_NAME} got interrupted.")
        stop_node()
