---
title: Adding and modifying sites
page_id: dev_sites
---

The `crazyflie-testing` infrastructures works by applying operations or tests on a collection of devices, called *sites*.
A site is defined by a *toml* file, following a certain format.

All site definition files are place in the `sites/` directory, and the files should end in `.toml`. In order to make use of them you set the `CRAZY_SITE` environment variable, for example:

```bash
CRAZY_SITE=single-cf python3 management/program --file my-firmware.bin
```

To flash all devices in a site with the `my-firmware.bin` file, or:

```bash
CRAZY_SITE=crazylab-malmö pytest tests/QA/test_param.py
```

To run the tests in `test_param.py` on all devices in the `crazylab-malmö` site.
Note that you should emit the `.toml` extension when setting the `CRAZY_SITE` variable.

## Site definition format

Let's look at an annotated part of the `crazylab-malmö.toml` site definition:

```toml
version = 1  # We have a version set so we can change the format safely in the
             # future, if needed,

[device.cf2_flow2_lighthouse] # The syntax here is device.[unique_id]

# A device must have have an URI where we can reach it
radio = "radio://0/50/2M/E7E7E71706"

# A device _can_ have decks, could be empty
decks = ["bcFlow2", "bcLighthouse4"]

# A device can have properties, could be empty
properties = ["powerMeasurement", "wifi"]

# A device _may_ have a bootloader URI, if it has we can use it to try to recover
# from bootloader mode.
bootloader_radio = "radio://0/0/2M/B19CF77F05?safelink=0"

[device.cf2_flow2]
radio = "radio://0/50/2M/E7E7E71705"
decks = ["bcFlow2"]
bootloader_radio = "radio://0/0/2M/B16298A8A3?safelink=0"

[device.cf2_flow2_multiranger]
radio = "radio://0/50/2M/E7E7E71704"
decks = ["bcFlow2", "bcMultiranger"]
bootloader_radio = "radio://0/0/2M/B1CEE678C5?safelink=0"

[...]
```

Using this site definition the infrastructure in this repository can make sure that operations like flashing or bootloader recovery as well as running test cases can be done for all devices defined.

## Adding your own site

Adding your own site is done by creating a `.toml` file with the format described above. With a device-entry for each device in your site, and dropping it in the `sites/` directory of this repository. After that you can set the  `CRAZY_SITE` environment variable to the name of your site (omitting the `.toml` extension).
