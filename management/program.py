# Copyright (C) 2021 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
from pathlib import Path

import argparse
import logging
import os
import sys
import traceback
import threading
import signal
import sys
import time

# Timeout for the program operation.
TIMEOUT = 10 * 60

#
# This is to make it possible to import from conftest
#
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.join(currentdir, '..')
sys.path.append(parentdir)

from conftest import get_devices  # noqa

logger = logging.getLogger(__name__)

current_frame = 0
latest_logging = 0

def alarm_handler(signum, frame):
    print('Timeout!')

    for th in threading.enumerate():
        print(th)
        traceback.print_stack(sys._current_frames()[th.ident], file=sys.stdout)
        print()

    sys.exit(-1)

def progress_cb(msg: str, percent: int):
    global current_frame
    global latest_logging

    if time.time() > (latest_logging + 3.0):
        frames = ['◢', '◣', '◤', '◥']
        frame = frames[current_frame % 4]

        print('{} {}% {}'.format(frame, percent, msg), end='\r')
        latest_logging = time.time()
    current_frame += 1


def program(fw_file: Path) -> bool:
    # Setup the alarm handler and ask the OS to send a SIGALRM to the process after TIMEOUT seconds
    signal.signal(signal.SIGALRM, alarm_handler)

    for dev in get_devices():
        try:
            signal.alarm(TIMEOUT)
            print('Programming device: {}'.format(dev))
            dev.flash(fw_file, progress_cb)
            signal.alarm(0)
        except Exception as err:
            signal.alarm(0)
            print('Programming failed: {}'.format(str(err)), file=sys.stderr)
            traceback.print_exc()
            return False

    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Flash firmware to all devices in site')
    parser.add_argument('--file', type=Path, help='Path to firmware file', required=True)
    p = parser.parse_args()

    if not program(p.file):
        sys.exit(1)
