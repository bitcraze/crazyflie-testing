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


def recover(name: str):
    for dev in get_devices():
        if not name or (name and dev.name == name):
            if not dev.recover():
                print(f'Failed to recover {dev.name}', file=sys.stderr)
            else:
                print(f'Recovered {dev.name} from bootloader mode')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Recover devices from bootloader mode')
    parser.add_argument('--name', type=Path, help='device to recover')
    p = parser.parse_args()

    recover(p.name)
