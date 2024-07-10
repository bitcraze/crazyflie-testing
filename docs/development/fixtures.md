---
title: Grokking the test fixtures
page_id: dev_fixtures
---

When creating tests using this infrastructure you write the test as if it was aimed at a single device. We then leverege the [pytest](https://docs.pytest.org/en/7.0.x/) framework to write [test fixtures](https://docs.pytest.org/en/7.0.x/how-to/fixtures.html#how-to-fixtures) that make the test run for each device in the specified [site](sites.md).

The fixtures are defined in the `conftest.py` script, in the root of the repository, the fixture is defined, at the time of writing, as below.
This documentation might not keep up with changes, but it aims to provide a bit of understanding of the principle in play.

```python
class DeviceFixture:
    def __init__(self, dev: BCDevice):
        self._device = dev

    @property
    def device(self) -> BCDevice:
        return self._device

    @property
    def kalman_active(self) -> bool:
        kalman_decks = ['bcLighthouse4', 'bcFlow', 'bcFlow2', 'bcDWM1000']
        if self._device.decks:
            return all(deck in kalman_decks for deck in self._device.decks)
        else:
            return False


@pytest.fixture
def test_setup(request):
    ''' This code will run before (and after) a test '''
    fix = DeviceFixture(request.param)
    yield fix  # code after this point will run as teardown after test
    fix.device.cf.close_link()
```

And in the the tests, for instance in `test_param.py`, we make use of the fixture like:

```python
@pytest.mark.parametrize(
    'test_setup',
    conftest.get_devices(),
    indirect=True,
    ids=lambda d: d.name
)
class TestParameters:
    def test_param_ronly(self, test_setup):

[...]
```

The fixture concept of **parametrize** basicly means **run the test for all the combinations we create here**. So, what this translates to is that we want to run the tests in the class `TestParameters` for all the devices returned by `conftest.get_devices()`. The `indirect=True` parameter means that we want to *preprocess* the device before we pass it as an argument to test. We pass it to the `test_setup` function above. The steps outlined are, for each test:

1. Get the list of devices defined in a [site](sites.md)
2. Get the properties and decks required for the test and check if the deck has them. If not, do not generate a test
3. For each device in the list ...
   * ... pass device as an argument to `test_setup` and get a `DeviceFixure` class ...
   * ... run the test methods with `DeviceFixture` as a fixture ...
   * ... which means we will get the class as an argument and that the code after `yield` will be run when each test is complete
