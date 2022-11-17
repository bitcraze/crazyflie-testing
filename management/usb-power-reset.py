#!/usr/bin/env python3

import argparse
from collections import namedtuple
import importlib
import sys
from typing import List
from pathlib import Path
from threading import Thread
import time


ROOT_PATH = Path(__file__).absolute().parent.parent
sys.path.append(str(ROOT_PATH))

# Import conftest and import get_devices() method so we can access usb_uri
# from the devices.
conftest = importlib.import_module('conftest')
get_devices = getattr(conftest, 'get_devices')
USB_Power_Control = getattr(conftest, 'USB_Power_Control')
USB_Power_Control_Action = getattr(conftest, 'USB_Power_Control_Action')


class USB_PowerControlThread(Thread):

    def __init__(self,
                 usb_power_controller: USB_Power_Control,
                 action: str
                ) -> None:
        super().__init__()
        self._usb_power_controller = usb_power_controller
        self._action = action

    def run(self) -> None:
        self._usb_power_controller.set_usb_power(self._action)


def run_action(usb_power_control: List[USB_Power_Control], action: str,
               power_on_delay: float = 0) -> None:
    print(f'Running action "{action}" for the usb ports.')
    # We'll set each usb port in a separate thread to make it faster,
    # since each command with uhubctl blocks for some time.
    usb_threads = [USB_PowerControlThread(ctrl, action) for ctrl in usb_power_control]

    for thread in usb_threads:
        thread.start()
        if action == USB_Power_Control_Action.ON:
            time.sleep(power_on_delay)

    for thread in usb_threads:
        thread.join()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--action', help='What action to perform,'
                        ' options: {on, off, toggle, reset}.',
                        default='on')
    parser.add_argument('-d', '--delay', help='Time (in seconds) delay'
                        ' between setting the power for different usb ports.',
                        default=0.3, type=float)
    return parser.parse_args()


if __name__ == '__main__':
    # Parse cli args.
    args = parse_args()
    action = args.action
    power_on_delay = args.delay

    # Get all devices from config reader.
    devices = get_devices()

    # Filter devices to get only devices that has usb power control configured.
    usb_power_control_devices = list(filter(lambda dev: dev.usb_power_control is not None, devices))
    print(f'Found {len(usb_power_control_devices)} usb ports to toggle:')
    print('\n'.join(f'\t{port}' for port in usb_power_control_devices))

    # Run the action for the devices.
    if action == USB_Power_Control_Action.RESET:
        run_action(usb_power_control_devices, USB_Power_Control_Action.OFF)
        run_action(usb_power_control_devices, USB_Power_Control_Action.ON, power_on_delay)
    else:
        run_action(usb_power_control_devices, action, power_on_delay)

    print('Done')
