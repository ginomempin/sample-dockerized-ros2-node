"""
App Tester Node

This is used to test the App Node.
"""


###############################################################################
# IMPORTS
###############################################################################


import os
import sys
from asyncio import Future
from random import randint
from time import sleep

import rclpy
from rclpy.qos import qos_profile_sensor_data

from myapp_apis.msg import AppCommand
from myapp_apis.srv import GetAppData


###############################################################################
# CONSTANTS
###############################################################################


NODE_NAME = "APP_TESTER"


###############################################################################
# CALLBACKS
###############################################################################


def get_app_data_callback(task: Future):
    data = task.result().data  # see return type in GetAppData.srv

    print(f"APP TESTER received: x={data.x}, y={data.y}, z={data.z}")


###############################################################################
# MAIN
###############################################################################


def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node(NODE_NAME)

    node_pub = node.create_publisher(
        AppCommand,                 # msg_type
        "send_app_command",         # topic
        qos_profile_sensor_data     # QoS
    )

    get_app_data = node.create_client(
        GetAppData,                 # srv_type
        "get_app_data"              # srv_name
    )
    while not get_app_data.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn("Unable to get 'get_app_data' service.")

    print(f"{NODE_NAME} is UP.")

    num_msgs = 0
    max_msgs = int(os.environ.get("APP_TESTER_NUM_MSGS", 5))
    print(f"{NODE_NAME} will send {max_msgs} msgs")

    cmd = AppCommand()
    while num_msgs < max_msgs:
        # publish to topic
        cmd.x = float(randint(10, 20))
        cmd.y = float(randint(20, 30))
        cmd.z = float(randint(30, 40))
        cmd.action = "hello!"
        print(f"{NODE_NAME} sent: x={cmd.x},y={cmd.y},z={cmd.z}")
        print(f"{NODE_NAME} sent: action={cmd.action}")
        node_pub.publish(cmd)

        # call service
        req = GetAppData.Request()
        req.user_id = randint(1, 5)
        rep = get_app_data.call_async(req)
        rep.add_done_callback(get_app_data_callback)
        print(f"{NODE_NAME} requested for user_id={req.user_id}")
        rclpy.spin_until_future_complete(node, rep)

        sleep(1)
        num_msgs += 1

    print(f"{NODE_NAME} is DOWN.")

if __name__ == '__main__':
    main()
