<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: BSD-3-Clause
)

[comment]: # (
SPDX-FileCopyrightText: 2021 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

# nav_msg_listener

Simple application that retrieves decoded navigation messages produced by
GNSS-SDR and prints them in a terminal. This is only for demonstration purposes,
as a example on how to retrieve data using the `nav_message.proto` file.

# Build the software

This software requires [Boost](https://www.boost.org/) and
[Protocol Buffers](https://developers.google.com/protocol-buffers).

In a terminal, type:

```
$ mkdir build && cd build
$ cmake ..
$ make
```

## Usage

In order to tell GNSS-SDR to generate those messages, you need to include the
lines:

```
NavDataMonitor.enable_monitor=true
NavDataMonitor.client_addresses=127.0.0.1  ; destination IP
NavDataMonitor.port=1237                   ; destination port
```

in your gnss-sdr configuration file. You can specify multiple destination
addresses, separated by underscores:

```
NavDataMonitor.client_addresses=79.154.253.31_79.154.253.32
```

Run gnss-sdr with your configuration, and at the same time, from the computer of
the client address (or another terminal from the same computer that is executing
gnss-sdr if you are using `127.0.0.1`), execute the binary as:

```
$ ./nav_msg_listener 1237
```

where `1237` needs to be the same port as in `NavDataMonitor.port`. As soon as
gnss-sdr starts to decode navigation messages, you will see them in your
terminal:

```
$ ./nav_msg_listener 1237

New Data received:
System: E
Signal: 1B
PRN: 11
TOW of last symbol [ms]: 75869044
Nav message: 000000001001010101010101010101010101010101010101010101010101010101010101010101010101010101010101010011100101110001000000

New Data received:
System: G
Signal: 1C
PRN: 16
TOW of last symbol [ms]: 75870000
Nav message: 100010111010101010101000101111000110001011001010010100011100010001000000000000000000000011010100000000101101100001000011000000000000000000000000111111101000010000110110011011000100000101111100000111100110110101000100110100100010011011101001001010011001011111111110000110000000000000000000000010001100

New Data received:
System: E
Signal: 5X
PRN: 18
TOW of last symbol [ms]: 75870260
Nav message: 0000100001111110010000010111110100011010010000100000000000000000000000000000000000000000000000000000000010101010000001001011010010100100100100100110101110110101010000100000000000000000111001011100010010100001010100001110101001001101111000000000

New Data received:
System: G
Signal: L5
PRN: 6
TOW of last symbol [ms]: 75871320
Nav message: 100010110001100011110001100010110010100111100001110100001000000110110101100101011100110111001101100001011001110110010100101110001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000001010101010111110000000

```
