#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse

import rclpy
from rclpy.node import Node

from charger_fleet_msgs.msg import ChargerMode
from charger_fleet_msgs.msg import ChargerRequest


def main(argv = sys.argv):
    '''
    Example charge request:
    - fleet_name: magni
    - charger_name: magni123
    - request_id: 6tyghb4edujrefyd
    - charger_mode.mode: CHARGE
    '''

    default_fleet_name = 'amr_vdm'
    default_charger_name = 'charger001'
    default_robot_name = 'amr_001'
    default_request_id = '576y13ewgyffeijuais'
    default_mode = 'charge'
    default_topic_name = '/charger_request'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-c', '--charger-name', default=default_charger_name)
    parser.add_argument('-r', '--robot-name', default=default_robot_name)
    parser.add_argument('-m', '--mode', default=default_mode)
    parser.add_argument('-i', '--request-id', default=default_request_id)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('charger_name: {}'.format(args.charger_name))
    print('robot_name: {}'.format(args.robot_name))
    print('mode: {}'.format(args.mode))
    print('request_id: {}'.format(args.request_id))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_charge_request_node')
    pub = node.create_publisher(ChargerRequest, args.topic_name, 10)

    msg = ChargerRequest()
    msg.fleet_name = args.fleet_name
    msg.charger_name = args.charger_name
    msg.robot_name = args.robot_name
    msg.request_id = args.request_id
    
    if args.mode == 'mode':
        print('Please insert desired mode: charge, uncharge, dropoff or uncharge')
        return
    elif args.mode == 'charge':
        msg.charger_mode.mode = ChargerMode.MODE_CHARGE 
    elif args.mode == 'uncharge':
        msg.charger_mode.mode = ChargerMode.MODE_UNCHARGE
    else:
        print('unrecognized mode requested, only use charge or uncharge please')
        return
  
    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
