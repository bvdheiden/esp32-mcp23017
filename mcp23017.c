/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
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

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/i2c.h>

#include <mcp23017.h>

#define REG_IODIR 0x00
#define REG_IOPOL 0x01
#define REG_GPINTEN 0x02
#define REG_DEFVAL 0x03
#define REG_INTCON 0x04
#define REG_IOCON 0x05
#define REG_GPPU 0x06
#define REG_INTF 0x07
#define REG_INTCAP 0x08
#define REG_GPIO 0x09
#define REG_OLAT 0x0A

#define SET_BIT(bit, value) ((value & 0x01) << bit)

/* --- IOCON configuration --- */
#define BANK_BIT 7
#define BANK_SEPERATE 1
#define BANK_SAME 0

#define MIRROR_BIT 6
#define MIRROR_CONNECTED 1
#define MIRROR_DISCONNECTED 0

#define SEQOP_BIT 5
#define SEQOP_DISABLED 1
#define SEQOP_ENABLED 0

#define DISSLW_BIT 4
#define DISSLW_DISABLED 1
#define DISSLW_ENABLED 0

#define HAEN_BIT 3
#define HAEN_ENABLED 1
#define HAEN_DISABLED 0

#define ODR_BIT 2
#define ODR_OPEN_DRAIN 1
#define ODR_ACTIVE_DRIVER 0

#define INTPOL_BIT 1
#define INTPOL_HIGH 1
#define INTPOL_LOW 0
/* --- IOCON configuration --- */

static esp_err_t _mcp23017_device_init(mcp23017_info_t *mcp23017_info);
static uint8_t _mcp23017_select_command(uint8_t command, mcp23017_port_t port);
static esp_err_t _mcp23017_read_port(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t *value, uint8_t command);
static esp_err_t _mcp23017_read_pin(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t *value, uint8_t command);
static esp_err_t _mcp23017_write_port(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value, uint8_t command);
static esp_err_t _mcp23017_write_pin(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value, uint8_t command);

static const char TAG[] = "mcp23017";

static esp_err_t _mcp23017_device_init(mcp23017_info_t *mcp23017_info)
{
    uint8_t data = 0;

    data |= SET_BIT(BANK_BIT, BANK_SEPERATE);
    data |= SET_BIT(MIRROR_BIT, MIRROR_DISCONNECTED);
    data |= SET_BIT(SEQOP_BIT, SEQOP_ENABLED);
    data |= SET_BIT(DISSLW_BIT, DISSLW_ENABLED);
    data |= SET_BIT(HAEN_BIT, HAEN_DISABLED);
    data |= SET_BIT(ODR_BIT, ODR_ACTIVE_DRIVER);
    data |= SET_BIT(INTPOL_BIT, INTPOL_LOW);

    return _mcp23017_write_port(mcp23017_info, PORTA, data, REG_IOCON);
}

static uint8_t _mcp23017_select_command(uint8_t command, mcp23017_port_t port)
{
    if (command == REG_IOCON || port == PORTA)
        return command;
    return command |= 0x10;
}

static esp_err_t _mcp23017_read_port(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t *value, uint8_t command)
{
    command = _mcp23017_select_command(command, port);

    ESP_LOGD(TAG, "reading command %d from port %d", command, port);

    xSemaphoreTake(*mcp23017_info->semaphore, portMAX_DELAY);

    i2c_cmd_handle_t link = i2c_cmd_link_create();
    i2c_master_start(link);
    i2c_master_write_byte(link, mcp23017_info->i2c_address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, command, true);
    i2c_master_stop(link);
    esp_err_t error = i2c_master_cmd_begin(mcp23017_info->i2c_port, link, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(link);

    link = i2c_cmd_link_create();
    i2c_master_start(link);
    i2c_master_write_byte(link, mcp23017_info->i2c_address << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(link, value, true);
    i2c_master_stop(link);
    error = i2c_master_cmd_begin(mcp23017_info->i2c_port, link, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(link);

    xSemaphoreGive(*mcp23017_info->semaphore);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to read value of port %d", port);
    }
    else
    {
        ESP_LOGD(TAG, "read %d from port %d with command 0x%02X", *value, port, command);
    }

    return error;
}

static esp_err_t _mcp23017_read_pin(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t *value, uint8_t command)
{
    uint8_t buffer = 0;
    esp_err_t error = _mcp23017_read_port(mcp23017_info, port, &buffer, command);
    if (error == ESP_OK)
    {
        buffer = (buffer >> pin) & 0x01;
        *value = buffer;
    }

    return error;
}

static esp_err_t _mcp23017_write_port(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value, uint8_t command)
{
    command = _mcp23017_select_command(command, port);

    xSemaphoreTake(*mcp23017_info->semaphore, portMAX_DELAY);

    i2c_cmd_handle_t link = i2c_cmd_link_create();
    i2c_master_start(link);
    i2c_master_write_byte(link, mcp23017_info->i2c_address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, command, true);
    i2c_master_write_byte(link, value, true);
    i2c_master_stop(link);
    esp_err_t error = i2c_master_cmd_begin(mcp23017_info->i2c_port, link, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(link);

    xSemaphoreGive(*mcp23017_info->semaphore);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to write value %d to port %d with command 0x%02X", value, port, command);
    }
    else
    {
        ESP_LOGD(TAG, "written %d to port %d with command 0x%02X", value, port, command);
    }

    return error;
}

static esp_err_t _mcp23017_write_pin(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value, uint8_t command)
{
    uint8_t buffer = 0;
    esp_err_t error = _mcp23017_read_port(mcp23017_info, port, &buffer, command);

    if (error == ESP_OK)
    {
        buffer &= (0xFF) ^ SET_BIT(pin, 1); // set pin bit low
        buffer |= SET_BIT(pin, value);      // set pin bit to value
        error = _mcp23017_write_port(mcp23017_info, port, buffer, command);
    }

    return error;
}

mcp23017_info_t *mcp23017_malloc(void)
{
    mcp23017_info_t *mcp23017_info = malloc(sizeof(*mcp23017_info));
    if (mcp23017_info != NULL)
    {
        memset(mcp23017_info, 0, sizeof(*mcp23017_info));
        ESP_LOGD(TAG, "malloc mcp23017_info_t %p", mcp23017_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc mcp23017_info_t failed");
    }

    return mcp23017_info;
}

void mcp23017_free(mcp23017_info_t **mcp23017_info)
{
    if (mcp23017_info != NULL && (*mcp23017_info != NULL))
    {
        ESP_LOGD(TAG, "free mcp23017_info_t %p", *mcp23017_info);
        free(*mcp23017_info);
        *mcp23017_info = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "free mcp23017_info_t failed");
    }
}

esp_err_t mcp23017_init(mcp23017_info_t *mcp23017_info, uint8_t i2c_port, uint8_t i2c_address, SemaphoreHandle_t *semaphore)
{
    esp_err_t error = ESP_FAIL;
    if (mcp23017_info != NULL)
    {
        mcp23017_info->i2c_port = i2c_port;
        mcp23017_info->i2c_address = i2c_address;
        mcp23017_info->semaphore = semaphore;
        mcp23017_info->init = true;

        error = _mcp23017_device_init(mcp23017_info);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "mcp23017 device init failed");
        }

        uint8_t buffer;
        error = _mcp23017_read_port(mcp23017_info, PORTA, &buffer, REG_IOCON);
        for (int i = 0; i < 8; i++)
        {
            ESP_LOGI(TAG, "IOCON bit %d = %d", i, (buffer >> i) & 1);
        }
    }
    else
    {
        ESP_LOGE(TAG, "mcp23017_info_t is NULL");
    }

    return error;
}

esp_err_t mcp23017_port_read(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t *value)
{
    return _mcp23017_read_port(mcp23017_info, port, value, REG_GPIO);
}

esp_err_t mcp23017_pin_read(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t *value)
{
    return _mcp23017_read_pin(mcp23017_info, port, pin, value, REG_GPIO);
}

esp_err_t mcp23017_port_write(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_GPIO);
}

esp_err_t mcp23017_pin_write(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_GPIO);
}

esp_err_t mcp23017_port_set_direction(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_IODIR);
}

esp_err_t mcp23017_pin_set_direction(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_iodir_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_IODIR);
}

esp_err_t mcp23017_port_set_polarity(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_IOPOL);
}

esp_err_t mcp23017_pin_set_polarity(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_ipol_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_IOPOL);
}

esp_err_t mcp23017_port_set_interrupt(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_GPINTEN);
}

esp_err_t mcp23017_pin_set_interrupt(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_gpinten_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_GPINTEN);
}

esp_err_t mcp23017_port_set_defval(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_DEFVAL);
}

esp_err_t mcp23017_pin_set_defval(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, uint8_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_DEFVAL);
}

esp_err_t mcp23017_port_set_compare(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_INTCON);
}

esp_err_t mcp23017_pin_set_compare(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_intcon_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_INTCON);
}

esp_err_t mcp23017_port_set_pullup(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_GPPU);
}

esp_err_t mcp23017_pin_set_pullup(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_gppu_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_GPPU);
}

esp_err_t mcp23017_port_set_latch(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t value)
{
    return _mcp23017_write_port(mcp23017_info, port, value, REG_OLAT);
}

esp_err_t mcp23017_pin_set_latch(mcp23017_info_t *mcp23017_info, mcp23017_port_t port, uint8_t pin, mcp23017_olat_t value)
{
    return _mcp23017_write_pin(mcp23017_info, port, pin, value, REG_OLAT);
}