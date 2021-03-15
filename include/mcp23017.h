/*
 * MIT License
 *
 * Copyright (c) 2021 Ben van der Heiden
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MCP23017_H
#define MCP23017_H

#include <esp_err.h>
#include <freertos/task.h>

typedef struct
{
    bool init;
    uint8_t i2c_port;
    uint8_t i2c_address;
    SemaphoreHandle_t *semaphore;
} mcp23017_info_t;

typedef enum
{
    PORTA,
    PORTB,
} mcp23017_port_t;

typedef enum
{
    MCP23017_DIRECTION_OUTPUT,
    MCP23017_DIRECTION_INPUT
} mcp23017_iodir_t;

typedef enum
{
    MCP23017_POLARITY_OPPOSITE,
    MCP23017_POLARITY_SAME
} mcp23017_ipol_t;

typedef enum
{
    MCP23017_INTERRUPT_ENABLE,
    MCP23017_INTERRUPT_DISABLE
} mcp23017_gpinten_t;

typedef enum
{
    MCP23017_COMPARE_DEFVAL,
    MCP23017_COMPARE_PREVIOUS
} mcp23017_intcon_t;

typedef enum
{
    MCP23017_PULLUP_ENABLED,
    MCP23017_PULLUP_DISABLED
} mcp23017_gppu_t;

typedef enum
{
    MCP23017_LATCH_HIGH,
    MCP23017_LATCH_LOW
} mcp23017_olat_t;

/**
 *  @brief  Construct a new MCP23017 info instance.
 *          New instance should be initialised before calling other functions.
 *  @return Pointer to new device info instance, or NULL if it cannot be created.
 */
mcp23017_info_t *mcp23017_malloc(void);

/**
 *  @brief  Delete an existing MCP23017 info instance.
 *  @param[in,out] mcp23017_info Pointer to MCP23017 info instance that will be freed and set to NULL.
 */
void mcp23017_free(mcp23017_info_t **mcp23017_info);

/**
 *  @brief  Initialize the MCP23017 info instance and configure the module.
 *  @param[in] mcp23017_info MCP23017 info instance that will be initialized.
 *  @param[in] i2c_port I2C port that will be used.
 *  @param[in] i2c_address I2C address of the MCP23017 module.
 *  @param[in] semaphore I2C semaphore handle to enable thread safe operations.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_init(mcp23017_info_t *mcp23017_info, uint8_t i2c_port, uint8_t i2c_address, SemaphoreHandle_t *semaphore);

/**
 *  @brief  Read a 8-bit value of a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to read value from.
 *  @param[in,out] value Pointer to the value variable that the value will be written to.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_read(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t *value);

/**
 *  @brief  Read a 1-bit value of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to read value from.
 *  @param[in] pin Pin to read value from.
 *  @param[in,out] value Pointer to the value variable that the value will be written to.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_read(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t *value);

/**
 *  @brief  Write a 8-bit value to a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_write(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Write a 1-bit value to a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_write(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value);

/**
 *  @brief  Set the direction of a GPIO port.
 *          [0 = output, 1 = input]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_direction(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the direction of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_direction(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_iodir_t value);

/**
 *  @brief  Set the polarity of a GPIO port.
 *          [0 = opposite, 1 = same]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_polarity(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the polarity of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_polarity(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_ipol_t value);

/**
 *  @brief  Set the interrupt mode of a GPIO port.
 *          [0 = enable, 1 = disable]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_interrupt(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the interrupt mode of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_interrupt(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_gpinten_t value);

/**
 *  @brief  Set the interrupt compare defval of a GPIO port.
 *          This can be used for generating interrupts when INTCON (comparison) is set to DEFVAL.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_defval(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the interrupt compare defval of a GPIO pin on a GPIO port.
 *          This can be used for generating interrupts when INTCON (comparison) is set to DEFVAL.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_defval(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value);

/**
 *  @brief  Set the interrupt compare mode of a GPIO port.
 *          [0 = defval, 1 = previous]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_compare(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the interrupt compare mode of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_compare(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_intcon_t value);

/**
 *  @brief  Set the pullup resistor mode of a GPIO port.
 *          [0 = enabled, 1 = disabled]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_pullup(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the pullup resistor mode of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_pullup(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_gppu_t value);

/**
 *  @brief  Set the output latch of a GPIO port.
 *          [0 = logic low, 1 = logic high]
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_port_set_latch(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value);

/**
 *  @brief  Set the output latch of a GPIO pin on a GPIO port.
 *  @param[in] mcp23017_info MCP23017 info instance that will be used.
 *  @param[in] port Port to write value to.
 *  @param[in] pin Pin to write value to.
 *  @param[in] value Value to write.
 *  @return Error type, should be ESP_OK when successful.
 */
esp_err_t mcp23017_pin_set_latch(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_olat_t value);

#endif