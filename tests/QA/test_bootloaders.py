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
import pytest
import conftest
import time



class TestBootloaders:

    @staticmethod
    def bootloader_back_and_forth(dev: conftest.BCDevice):
        # The start_bootloader method only returns true if it can communicate with the bootloader.
        assert dev.bl.start_bootloader(warm_boot=True)
        dev.bl.reset_to_firmware()

        # Give the CF some time to boot
        time.sleep(0.5)
        assert dev.firmware_up()

        dev.bl.close()

    def test_bootloader_reset_simple(self, dev: conftest.BCDevice):
        self.bootloader_back_and_forth(dev)

    def test_bootloader_reset_stress(self, dev: conftest.BCDevice):
       requirement = conftest.get_requirement('bootloaders.reliability')
       for _ in range(0, requirement['iterations']):
           self.bootloader_back_and_forth(dev)
