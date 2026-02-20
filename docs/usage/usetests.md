---
title: Running tests
page_id: use_tests
---

The tests in this repository is written using the [pytest](https://docs.pytest.org/en/7.0.x/) framework and they are run against specific [sites](/docs/development/sites.md). A site is a collection of devices with specific properties and capabilities. The available sites are defined in the `sites/` folder.

In order to run tests against a site you need to set the `CRAZY_SITE` environment variable before invocing `pytest`:

```bash
$ CRAZY_SITE=single-cf pytest -s suites/cflib2/
```

Please note that the `CRAZY_SITE` variable should point out a **name** of a site, not the path or a filename.

## Where are the tests?

Tests are organized by the library they target. Tests using cflib2 are found in the `suites/cflib2` folder and run automatically by sites such as the [Crazylab](https://www.bitcraze.io/2021/08/the-beginnings-of-a-test-lab/). Tests that require flying or human intervention should not be put there.

There are many ways to run pytest, to select or deselect, or make other configurations of how to run tests. For all of those we delegate to the [Pytest documentation](https://docs.pytest.org/en/6.2.x/usage.html).
