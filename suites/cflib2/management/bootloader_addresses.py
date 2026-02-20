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
import logging
import os
import sys

#
# This is to make it possible to import from conftest
#
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.join(currentdir, '..')
sys.path.append(parentdir)

from conftest import BCDevice, get_devices, get_bl_address  # noqa

logger = logging.getLogger(__name__)


def list_addresses():
    for dev in get_devices():
        address = get_bl_address(dev)
        if address is None:
            print(f'{dev.name}: failed to get bootloader address')
            continue

        print(f'{dev.name}: radio://0/0/2M/{address}?safelink=0')


if __name__ == "__main__":
    list_addresses()
