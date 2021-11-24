# Pronto HEX infrared code transceiver

This project aims to deliver an STM32 firmware that can record infrared codes from remote controllers
and send the Pronto HEX encoded string over UART, and can emit infrared codes from received strings.

The *Code* is written to be very portable across STM32's, to adapt to your hardware, just take over the Cube
configuration parameters, and adapt the configuration section at the top of `Code/infrared_config_stm32.h`.
To build the code, you will need to include the [Embedded Template Library][etl].

A very detailed project report is available on [hackaday.io][hackaday].

[hackaday]: https://hackaday.io/project/182577
[etl]: https://github.com/ETLCPP/etl
