



Contents
========

* [Radio](#radio)
	* [latencysmall](#latencysmall)
	* [latencybig](#latencybig)
	* [bwsmall](#bwsmall)
	* [bwbig](#bwbig)
	* [reliability](#reliability)


<!-- This file is auto-generated from: communication.toml -->


# Radio


These requirements targets the low level radio communication.
## latencysmall


Link round-trip latency for small radio packets (4 bytes)
|Field|Value|
| :--- | :--- |
|limit_high_ms|8|
|packet_size|4|
|rational|Empirical|

## latencybig


Link round-trip latency for small radio packets (28 bytes)
|Field|Value|
| :--- | :--- |
|limit_high_ms|8|
|packet_size|28|
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
|background|The design is that there should never be any packet loss ever. So we should be able to exchange an infinit amount of packet. However, testing infinity is hard so, as a compromise, we are testing on 30 000 packets which takes ~= 30s to test. |
|limit_low|30000|
|rational|Design|
