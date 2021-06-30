# esp32-mcp23017

ARCHIVED, DO NOT USE

## Introduction
This component provides an API to communicate with the MCP23017 module via the I2C protocol. It is written especially for the ESP32 IDF framework and will also work with the ADF framework since it does not reinstall the I2C drivers. It is thread-safe by requiring the user to pass a FreeRTOS semaphore mutex.

**This component does not have any dependencies on other components other than the provided components by the IDF framework**

## Usage
Initialization of the component
1. Install the I2C driver with compliance to the specifications listed by the MCP23017.
2. Malloc the `mcp23017_info_t` intance by using `mcp23017_malloc`.
3. Create a semaphore mutex.
4. Initialize the MCP23017 by using `mcp23017_init`.

Deinitialization of the component
1. Free the resources by using `mcp23017_free`

## Acknowledgements
* "I2C" is a registered trademark of Phillips Corporation.
* "IDF" is the Espressif IoT Development Framework
* "ADF" is the Espressif Audio Development Framework
