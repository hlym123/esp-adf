#include "i2c_device.h"

#include <esp_log.h>
#include <string.h>
#include <stdlib.h>

#define TAG "I2cDevice"


I2cDevice::I2cDevice(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_));
    assert(i2c_device_ != NULL);
}

void I2cDevice::WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buffer, 2, 100));
}

void I2cDevice::WriteRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    // Create a buffer that includes the register address followed by the data
    // Use stack allocation for small buffers, heap for larger ones
    const size_t MAX_STACK_SIZE = 64;
    uint8_t stack_buffer[MAX_STACK_SIZE + 1];
    uint8_t* write_buffer = nullptr;
    bool use_heap = (length + 1 > MAX_STACK_SIZE);
    
    if (use_heap) {
        write_buffer = (uint8_t*)malloc(length + 1);
        if (write_buffer == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate memory for WriteRegs");
            return;
        }
    } else {
        write_buffer = stack_buffer;
    }
    
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], buffer, length);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, write_buffer, length + 1, 100));
    
    if (use_heap) {
        free(write_buffer);
    }
}

uint8_t I2cDevice::ReadReg(uint8_t reg) {
    uint8_t buffer[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, 1, 100));
    return buffer[0];
}

void I2cDevice::ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, 100));
}