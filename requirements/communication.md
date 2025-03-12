



Contents
========

* [Radio](#radio)
	* [latency](#latency)
	* [bwsmall](#bwsmall)
	* [bwbig](#bwbig)
	* [reliability](#reliability)


<!-- This file is auto-generated from: communication.toml -->


# Radio


These requirements targets the low level radio communication.
## latency


Link round-trip latency for radio packets
|Field|Value|
| :--- | :--- |
|limit_high_ms|8|
|rational|Empirical|

## bwsmall


Packet rate (packets per seconds) for small radio packets (4 bytes)
|Field|Value|
| :--- | :--- |
|background|This is what our test machine manages through Docker. Which can be viewed as our lowest supported system. |
|limit_low|500|
|packet_size|4|
|rational|Empirical|

## bwbig


Packet rate (packets per seconds) for big radio packet (28 bytes)
|Field|Value|
| :--- | :--- |
|background|This is what our test machine manages through Docker. Which can be viewed as our lowest supported system. |
|limit_low|350|
|packet_size|28|
|rational|Empirical|

## reliability


Packet exchange without any information loss
|Field|Value|
| :--- | :--- |
|background|We are pinging the Crazyflie with the max size packet and never loose any data |
|limit_low|500|
|packet_size|30|
|rational|Design|
