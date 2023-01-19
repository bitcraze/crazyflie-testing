# Copyright (C) 2021 - 2023 Bitcraze AB
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
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


#
# Using the indirect=True parameter when parametrizing a test allows to
# parametrize a test with a fixture receiving the values before passing them to
# a test. In this case it means a device in the array returned from
# get_devices() will be passed to test_setup() in conftest.py before being used
# as a parameter in the test methods.
#
@pytest.mark.parametrize(
    'test_setup',
    conftest.get_devices(),
    indirect=True,
    ids=lambda d: d.name
)
class TestDecks:

    def test_deck_present(self, test_setup):
        '''
        Check that all decks defined for the device in the site
        is detected, using the parameter interface.
        '''
        if not test_setup.device.decks:
            pytest.skip('no decks on device')

        assert test_setup.device.connect_sync()

        for deck in test_setup.device.decks:
            is_deck_present = int(test_setup.device.cf.param.get_value(f'deck.{deck}'))
            assert is_deck_present


    def test_loco_deck_loop_is_running(self, test_setup):
        '''
        Check that the event loop in the loco deck driver is running, this is indicated by read and writes to the SPI
        bus.
        '''
        if not test_setup.has_loco_deck:
            pytest.skip('Only on loco decks')

        assert test_setup.device.connect_sync()

        READ_LOG = 'loco.spiRe'
        WRITE_LOG = 'loco.spiWr'

        log_config = LogConfig(name='locodeck', period_in_ms=10)
        log_config.add_variable(READ_LOG, 'float')
        log_config.add_variable(WRITE_LOG, 'float')

        with SyncLogger(test_setup.device.cf, log_config) as logger:
            for entry in logger:
                read_rate = entry[1][READ_LOG]
                write_rate = entry[1][WRITE_LOG]

                assert read_rate > 10.0
                assert write_rate > 10.0
                break
