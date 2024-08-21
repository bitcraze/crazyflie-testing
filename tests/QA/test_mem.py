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
from threading import Event
import pytest
import conftest
from cflib.crazyflie.mem import MemoryElement


class TestMem:
    def test_mem_ow(self, test_setup: conftest.DeviceFixture):
        '''
        Check that we can read the One Wire memory from decks and that it matches the deck.
        '''
        def data_updated_cb(mem):
            nonlocal read_mems
            read_mems.append(mem.elements['Board name'])
            update_event.set()

        assert test_setup.device.connect_sync()
        mems = test_setup.device.cf.mem.get_mems(MemoryElement.TYPE_1W)
        assert len(mems) == len(test_setup.device.decks)

        update_event = Event()
        read_mems = []
        for m in mems:
            update_event.clear()
            m.update(data_updated_cb)
            success = update_event.wait(timeout=1.0)
            assert success

        assert len(read_mems) == len(test_setup.device.decks)
        assert sorted(read_mems) == sorted(test_setup.device.decks)
