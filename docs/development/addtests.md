---
title: Adding tests
page_id: dev_addtests
---

The tests in this repository is written using the [pytest](https://docs.pytest.org/en/7.0.x/) framework and they make use of special [fixtures](fixtures.md) to integrate with the available test [sites](sites.md).

## Where to add tests?

Tests should be added under the `tests/` folder. Right now we have a convention that tests that are to be run automagicly by sites, such as the [Crazylab](https://www.bitcraze.io/2021/08/the-beginnings-of-a-test-lab/), should be put in the `tests/QA` folder. Tests that require flying or human intervention should not be put in the `QA` folder.

We also have a special folder `tests/crazyswarm` for tests that are special to the [Crazyswarm project](https://crazyswarm.readthedocs.io/en/latest/).


## Connecting to the Crazyflie

There are two main ways to connect to a Crazyflie in a test. These methods adds extra functionality that checks the
state of the Crazyflie after connection to make sure the self tests passed (that it has not asserted) as well as
automatically starts collection of console logs. Note that console logs only are displayed when a test fails.

### Option 1 - using connect_sync():

Use the `connect_sync()` method.

``` python
# Make sure to assert the result of connect_sync()
assert test_setup.device.connect_sync()
# You can now access the connected Crazyflie through test_setup.device.cf
element = test_setup.device.cf.param.toc.get_element_by_complete_name('my.param')
# The connection is automatically closed when the test ends

```

### Option 2 - using ValidatedSyncCrazyflie

The `ValidatedSyncCrazyflie` class behaves like `SyncCrazyflie` but with added checks when connected.

``` python
with ValidatedSyncCrazyflie(test_setup.device.link_uri) as scf:
    # Do stuff with scf
    # The connection is closed when scf goes out of scope
```

## Example test
For an example of how an pytest, interacting with sites, can look, see below:

```python
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
        Check that all decks defined in for the device in the site
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
```
