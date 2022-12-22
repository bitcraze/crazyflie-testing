# crazyflie-testing
Tests and infrastructure for testing Bitcraze devices in a physical lab.

## Requirements
The tests defined in the QA suite (`tests/QA`) all reference requirements found
in the `requirements/` folder. The requiremens are written in `TOML` and are
parsed by the test suite in a way that the tests can reference limits in the
tests.

## Sites
Before running the test suite you need to define a site in the `sites/` folder.
The site `TOML` tells the test suite which devices to tests, what capabilities
and decks they have, and how to reach them.

See [site docmentation](docs/development/sites.md) for the site file format to define new test sites.
## Running the test

To run the test for a single Crazyflie, run:
```
CRAZY_SITE=single-cf pytest --verbose tests/QA
```

If you have defined your own site, then change the `CRAZY_SITE` environment
variable to reflect that. For more information see the [running tests documentation](docs/usetests.md).

## Management
There are some scripts in the `management/` folder to help manage the devices
in your site. For details see the [management documentation](docs/use_management.md).

## Testing with Crazyswarm
It also possible to test using the [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) project.
You will need to specify your swarm in `swarms/name.yaml` and a [ROS](https://www.ros.org/) launch file in `swarms/name.launch` you can check the `swarms/crazylab-malmö.[yaml|launch]` files for inspiration.

To run the Crazyswarm tests or flash firmware files to a swarm you need to define some environment variables:

```
$ export CRAZYSWARM_PATH=[path to checked out crazyswarm source]
$ export CRAZYSWARM_YAML=[name_of_your.yaml]

$ source $CRAZYSWARM_PATH/ros_ws/devel/setup.bash
$ roslaunch [path to your launch file] &

# To flash all devices in swarm
$ python3 management/program_swarm.py [firmware file]

# To run the crazyswarm tests
$ python3 -m pytest tests/crazyswarm
```
