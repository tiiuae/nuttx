> **TII fork, `skunkworks`.** Apache NuttX for the Saluki boards, carrying
> i.MX93 EdgeLock Enclave drivers. PX4 builds it only
> through [px4-firmware `skunkworks`](https://github.com/tiiuae/px4-firmware/tree/skunkworks#build),
> which pins this branch. Everything below is upstream's README.
>
> | Carried here                                | Upstream                                                   |
> | ------------------------------------------- | ---------------------------------------------------------- |
> | ELE-backed `/dev/random`                    | <https://github.com/apache/nuttx/pull/20191>, in review    |
> | ELE key store, signing, persistence         | <https://github.com/apache/nuttx/pull/20343>, in review    |
> | ELE physical and virtual addresses          | <https://github.com/apache/nuttx/pull/20196>, merged       |
>
> Change a driver upstream first when it is general, then carry it here.
> Do not rebase `skunkworks`; it is shared.

<p align="center">
<img src="https://raw.githubusercontent.com/apache/nuttx/master/Documentation/_static/NuttX320.png" width="175">
</p>

![POSIX Badge](https://img.shields.io/badge/POSIX-Compliant-brightgreen?style=flat&label=POSIX)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue
)](https://nuttx.apache.org/docs/latest/introduction/licensing.html)
![Issues Tracking Badge](https://img.shields.io/badge/issue_track-github-blue?style=flat&label=Issue%20Tracking)
[![Contributors](https://img.shields.io/github/contributors/apache/nuttx
)](https://github.com/apache/nuttx/graphs/contributors)
[![GitHub Build Badge](https://github.com/apache/nuttx/workflows/Build/badge.svg)](https://github.com/apache/nuttx/actions/workflows/build.yml)
[![Documentation Badge](https://github.com/apache/nuttx/workflows/Build%20Documentation/badge.svg)](https://nuttx.apache.org/docs/latest/index.html)

Apache NuttX is a real-time operating system (RTOS) with an emphasis on
standards compliance and small footprint. Scalable from 8-bit to 64-bit
microcontroller environments, the primary governing standards in NuttX are POSIX
and ANSI standards. Additional standard APIs from Unix and other common RTOSs
(such as VxWorks) are adopted for functionality not available under these
standards, or for functionality that is not appropriate for deeply-embedded
environments (such as fork()).

For brevity, many parts of the documentation will refer to Apache NuttX as simply NuttX.

## Getting Started
First time on NuttX? Read the [Getting Started](https://nuttx.apache.org/docs/latest/quickstart/index.html) guide!
If you don't have a board available, NuttX has its own simulator that you can run on terminal.

## Documentation
You can find the current NuttX documentation on the [Documentation Page](https://nuttx.apache.org/docs/latest/).

Alternatively, you can build the documentation yourself by following the Documentation Build [Instructions](https://nuttx.apache.org/docs/latest/contributing/documentation.html).

The old NuttX documentation is still available in the [Apache wiki](https://cwiki.apache.org/NUTTX/NuttX).

## Supported Boards
NuttX supports a wide variety of platforms. See the full list on the [Supported Platforms](https://nuttx.apache.org/docs/latest/platforms/index.html) page.

## Contributing
If you wish to contribute to the NuttX project, read the [Contributing](https://nuttx.apache.org/docs/latest/contributing/index.html) guidelines for information on Git usage, coding standard, workflow and the NuttX principles.

## License
The code in this repository is under either the Apache 2 license, or a license compatible with the Apache 2 license. See the [License Page](https://nuttx.apache.org/docs/latest/introduction/licensing.html) for more information.
