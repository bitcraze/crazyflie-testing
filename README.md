# crazyflie-testing
Tests and infrastructure for testing Bitcraze devices in a physical lab.

## Requirements
The tests in `suites/cflib2/` reference requirements found in the `requirements/`
folder. The requirements are written in `TOML` and are parsed by the test suite
so that tests can reference limits defined there.

## Sites
Before running the test suite you need to define a site in the `sites/` folder.
The site `TOML` tells the test suite which devices to tests, what capabilities
and decks they have, and how to reach them.

The default site will be single-cf

See [site docmentation](docs/development/sites.md) for the site file format to define new test sites.
## Running the test

To run the test for a single Crazyflie, run:
```
CRAZY_SITE=single-cf pytest --verbose suites/cflib2 -k test_filter
```

or specify the name(s) of crazyflies to run on
```
CRAZY_DEVICE=cf21_flow2_multiranger,cf21_flow2... pytest --verbose suites/cflib2 -k test_filter
```


If you have defined your own site, then change the `CRAZY_SITE` environment
variable to reflect that. For more information see the [running tests documentation](docs/usetests.md).

## Management
There are some scripts in the `management/` folder to help manage the devices
in your site. For details see the [management documentation](docs/use_management.md).

