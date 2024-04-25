#!/usr/bin/env python3

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
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

'''Set servo-level configuration for a quad A1 robot.'''


import argparse
import asyncio
import moteus
import moteus_pi3hat
import os
import subprocess
import sys
import tempfile


SCRIPT_PATH = os.path.dirname(__file__)

CONFIG = {
    'servopos.position_min' : 'nan',
    'servopos.position_max' : 'nan',
    'servo.flux_brake_min_voltage' : '20.5',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : '4',
    'servo.pid_position.kd' : '0.05',

    'aux2.spi.mode' : '1', # disabled
    'aux2.pins.0.mode' : '6',  # hall
    'aux2.pins.1.mode' : '6',  # hall
    'aux2.pins.2.mode' : '6',  # hall
    'aux2.pins.0.pull' : '1',  # pullup
    'aux2.pins.1.pull' : '1',  # pullup
    'aux2.pins.2.pull' : '1',  # pullup
    'aux2.hall.enabled' : '1',

    'motor_position.sources.0.aux_number' : '2',
    'motor_position.sources.0.type' : '4',
    'motor_position.sources.0.cpr' : '90',
    'motor_position.sources.0.offset' : '-4',
    'motor_position.sources.0.pll_filter_hz' : '45',
    'motor.poles' : '30',
    'motor.resistance_ohm' : '0.326',
    'motor.v_per_hz' : '1.293',
}

async def config_servo(args, transport, servo_id):
    if args.verbose:
        print(f"*** SERVO {servo_id} ***")

    c = moteus.Controller(id=servo_id, transport=transport)
    s = moteus.Stream(c, verbose=args.verbose)

    await s.flush_read()

    for key, data_or_value in CONFIG.items():
        try:
            if type(data_or_value) == str:
                value = data_or_value
                await s.command(
                    "conf set {} {}".format(key, value).encode('utf8'))
            else:
                for servo_selector, value in data_or_value:
                    ids = set([int(x) for x in servo_selector.split(',')])
                    if not servo_id in ids:
                        continue
                    await s.command(
                        "conf set {} {}".format(key, value).encode('utf8'))
        except:
            print(f"While setting {key} / {data_or_value}")
            raise

    await s.command(b'conf write')


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    if os.geteuid() != 0:
        raise RuntimeError('This must be run as root')

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1: [1],
            3: [2],
        },
    )

    for i in range(1, 3):
        await config_servo(args, transport, i)

if __name__ == '__main__':
    asyncio.run(main())
