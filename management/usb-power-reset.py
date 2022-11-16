#!/usr/bin/env python3

import subprocess
from collections import namedtuple
import argparse
import sys
from typing import List
from pathlib import Path
import importlib
from threading import Thread
import time


ROOT_PATH = Path(__file__).absolute().parent.parent
USB_Port = namedtuple('Port', ['hub', 'port'])
sys.path.append(str(ROOT_PATH))

# Import conftest and import get_devices() method so we can access usb_uri
# from the devices.
conftest    = importlib.import_module('conftest')
get_devices = getattr(conftest, 'get_devices')


class Action:
    ON     = 'on'
    OFF    = 'off'
    TOGGLE = 'toggle'
    RESET  = 'reset'


def set_usb_port(port: USB_Port, action: str) -> None:
    cmd = f'uhubctl -l {port.hub} -p {port.port} -a {action}'
    print(f'> {cmd}')
    pipe = subprocess.Popen(
        cmd.split(' '),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    out = pipe.stdout.read()
    err = pipe.stderr.read()

    if out:
        print(out.decode('utf-8'))
    if err:
        print(f'Error: {err.decode("utf-8")}')


def parse_uhub_uri(device: 'BCDevice') -> USB_Port:
    usb_uri = device.usb_uri
    if usb_uri is None:
        return None

    data = usb_uri.split('usb://')[-1]
    hub, port = data.split(' ')
    return USB_Port(hub, port)


def get_usb_ports() -> List[USB_Port]:
    devices = get_devices()
    usb_ports = map(parse_uhub_uri, devices)
    return list(filter(lambda port: port is not None, usb_ports))


class USB_Thread(Thread):

    def __init__(self, port: USB_Port, action: str) -> None:
        super().__init__()
        self._port = port
        self._action = action

    def run(self) -> None:
        set_usb_port(self._port, self._action)


def run_action(usb_ports: List[USB_Port], action: str,
               power_on_delay: float = 0) -> None:
    print(f'Running action {action} for the usb ports.')
    # We'll set each usb port in a separate thread to make it faster,
    # since each command with uhubctl blocks for some time.
    usb_threads = [USB_Thread(port, action) for port in usb_ports]

    for thread in usb_threads:
        thread.start()
        if action == Action.ON:
            time.sleep(power_on_delay)

    for thread in usb_threads:
        thread.join()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--action', help='What action to perform,'
                        ' options: {on, off, toggle, reset}.',
                        default=Action.ON)
    parser.add_argument('-d', '--delay', help='Time (in seconds) delay'
                        ' between setting the power for different usb ports.',
                        default=0.3, type=float)
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    action = args.action
    power_on_delay = args.delay

    usb_ports = get_usb_ports()

    print(f'Found USB ports to toggle: {len(usb_ports)}')
    print('\n'.join(f'\t{port}' for port in usb_ports))

    if action == Action.RESET:
        run_action(usb_ports, Action.OFF)
        run_action(usb_ports, Action.ON, power_on_delay)
    else:
        run_action(usb_ports, action, power_on_delay)

    print('Done')

