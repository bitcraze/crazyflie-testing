---
title: Adding tests
page_id: dev_addtests
---

The tests in this repository is written using the [pytest](https://docs.pytest.org/en/7.0.x/) framework and they make use of special [fixtures](fixtures.md) to integrate with the available test [sites](sites.md).

## Where to add tests?

Tests should be added under the `tests/` folder. Right now we have a convention that tests that are to be run automagicly by sites, such as the [Crazylab](https://www.bitcraze.io/2021/08/the-beginnings-of-a-test-lab/), should be put in the `tests/QA` folder. Tests that require flying or human intervention should not be put in the `QA` folder.

We also have a special folder `tests/crazyswarm` for tests that are special to the [Crazyswarm project](https://crazyswarm.readthedocs.io/en/latest/).


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
