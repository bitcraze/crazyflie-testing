---
title: The build and test process
page_id: build_process
---

We run tests in our stability test lab every night with the purpose to make sure the code base is stable and to catch
instabilities that only happens seldom.

Currently we run three sets of test in the lab: "Crazy Stab Lab", "Crazyswarm test" and "Crazy Stab Lab Experiment".
Implementations can be found in corresponding github action files in `.github/workflows`.

## Build process

The firmware that is used in tests is built from a number of repositories, compiled into a zip and finally flashed to
the target hardware (Crazyflies and Roadrunners). The build process is outlined bellow.

### Build of firmware repositories

Repositories containing firmware or other source code that is compiled into a binary is build on each commit. If the
build passes, the generated binaries are stored as build artifacts on the build job in github actions for that repository.
This process is used for instance on [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) and
[bitcraze/crazyflie2-nrf-firmware](https://github.com/bitcraze/crazyflie2-nrf-firmware).

### Nightly build of bitcraze/crazyflie-release

The [bitcraze/crazyflie-release](https://github.com/bitcraze/crazyflie-release) is used to compile a number of binaries
into a zip that can be flashed to a Crazyflie with decks to update all firmware in the system. In this repository there
is a nightly build that compiles the binaries from the latest build in the firmware repositories by downloading them
from github.
This is implemented in the [nightly.yml github action](https://github.com/bitcraze/crazyflie-release/blob/master/.github/workflows/nightly.yml)
and the [versions-nightly.json](https://github.com/bitcraze/crazyflie-release/blob/master/src/versions-nightly.json)
files defines the versions that included in the build.

The resulting zips that are generated are stored as artifacts on the build job.

### Running tests in the test lab

The last step in the chain is to use the generated zip to flash the hardware in the test lab. The zip is downloaded from
the latest nightly build of bitcraze/crazyflie-release in github and flashed through the test infrastructure in the lab.

### The Stab Lab Experiment test

There is a "special" track in the system for running experiments in the stab lab. The purpose is to make it possible
to try code variants without breaking master/main.

This track essentially works the same way as the normal Stab lab, with the exception that it uses the
[versions-nightly-experiment.json](https://github.com/bitcraze/crazyflie-release/blob/master/src/versions-nightly-experiment.json)
file to define the zip. The typical use case is to commit the code you want to run on a branch, for instance
`my-experiment-branch` in the crazyflie-firmware repo. Then modify the versions-nightly-experiment.json file to use
the latest version of the branch

``` json
{
  "crazyflie-release-version": "nightly-experiment",
  "crazyflie-firmware-version": "latest:my-experiment-branch",
  "crazyflie2-nrf-firmware-version": "latest:master",
  "lighthouse-fpga-version": "V6",
  "aideck-esp-firmware-version": "latest:main",
  "aideck-gap8-examples-version": "latest:master"
}
```

and that is it!

The experimental tests are run every night.

## Manually running builds/tests from github after a code change

If you want to manually run the tests in the test lab to verify a code change and you do not want to wait for the next
nightly run, you should:

1. Commit your code change to the appropriate repo
2. Wait for the build in the repo to finish
3. Run the nightly build in crazyflie-release manually
4. Run the appropriate action in this repository to execute the tests
