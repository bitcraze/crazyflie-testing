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

        discovered = list()

        def deck_param_callback(name, value):
            nonlocal discovered

            if int(value) == 1:
                discovered.append(name.rsplit('.')[-1])  # deck.name => name

        assert test_setup.device.connect_sync()

        test_setup.device.cf.param.add_update_callback(
            group='deck',
            name=None,
            cb=deck_param_callback
        )

        time.sleep(3)

        for deck in test_setup.device.decks:
            assert deck in discovered
