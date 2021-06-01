# crazylab-testing
Tests and infrastructure for testing Bitcraze products in a physical lab

## Running the test

To run the test for a single Crazyflie, run:
```
CRAZY_SITE=single-cf pytest --verbose
```

See the file `sites/single-cf.toml` for the site file format to define new test sites.
