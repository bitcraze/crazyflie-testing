



Contents
========

* [Logging](#logging)
	* [basic](#basic)
	* [variables](#variables)
	* [blocks](#blocks)
	* [rate](#rate)


<!-- This file is auto-generated from: logging.toml -->


# Logging


These requirements targets the Crazyflie logging framework.
## basic


It must be possible to receive logging variable values in both a synchronous and
asynchronous manner. It must also be possible to set a desired rate of how
often one receives updates of the logging variable.

|Field|Value|
| :--- | :--- |
|max_rate|100|
|rational|Design|

## variables


It should not be possible to add more than 128 variables to a log config.
|Field|Value|
| :--- | :--- |
|max|128|
|rational|Design|

## blocks


It should not be possible to add more than 16 log blocks to a log config. And
at most 26 bytes per block.

|Field|Value|
| :--- | :--- |
|max|16|
|max_payload|26|
|rational|Design|

## rate


Packet rate (packets per seconds) with full payloaded log blocks. And when no
heavy task (like kalman) is running.

|Field|Value|
| :--- | :--- |
|background|This is what our test machine manages through Docker. Which can be viewed as our lowest supported system. |
|limit_low|300|
|rational|Empirical|
