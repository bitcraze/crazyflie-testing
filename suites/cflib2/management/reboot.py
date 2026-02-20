from pathlib import Path

import argparse
import logging
import os
import sys

#
# This is to make it possible to import from conftest
#
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.join(currentdir, '..')
sys.path.append(parentdir)

from conftest import BCDevice, get_devices  # noqa

logger = logging.getLogger(__name__)


def reboot(name: str):
    for dev in get_devices():
        if not name or (name and dev.name == name):
            print(f'Rebooting {dev.name}')
            dev.reboot()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reboot devices')
    parser.add_argument('--name', type=Path, help='device to reboot')
    p = parser.parse_args()

    reboot(p.name)
