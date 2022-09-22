---
title: Managing sites
page_id: use_management
---

In addition to tests this repository also includes some management tools, that allow you to perform operations on all devices in a site.
They are located in the `management/` folder.

| tool                                 | description                                | usage                         |
| ------------------------------------ | -------------------------------------------| ----------------------------- |
| `program.py`              | Flash binary or zip file on all devices.              | `program.py --file [file]`    |
| `reboot.py`               | Reboot one or all devices.                            | `reboot.py [--name [device]]` |
| `recover.py`              | Attempt to recover one or all devices from bootloader.| `recover.py [--name [device]]`|
| `bootloader_addresses.py` | Get bootloader address from all devices.              | `bootloader_addresses.py`     |

To use this tools you need to specify what site to run against using the `CRAZY_SITE` environment variable.

## Example

```bash
$ CRAZY_SITE=crazylab-malmö python3 bootloader_addresses.py
cf2_flow2_lighthouse: radio://0/0/2M/B19CF77F05?safelink=0
cf2_flow2: failed to get bootloader address
cf2_flow2_multiranger: radio://0/0/2M/B1CEE678C5?safelink=0
cf2_stock: radio://0/0/2M/B177790F3A?safelink=0
```
