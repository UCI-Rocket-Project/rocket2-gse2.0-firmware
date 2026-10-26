#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <optional>

#include "main.h"

static int mock_transmit_status = HAL_OK;
static int mock_transmit_fail_count = 0;
static int mock_transmit_call_count = 0;
static std::vector<uint8_t> mock_transmitted_bytes;

static int mock_receive_status = HAL_OK;
static int mock_receive_call_count = 0;
static uint8_t mock_rx_source[16] = {0};
static uint8_t* mock_rx_dest = nullptr;
static uint16_t mock_rx_size = 0;

static int mock_delay_call_count = 0;

extern "C" {
    HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
        mock_transmit_call_count++;
        if (pData && Size > 0) {
            for (uint16_t i = 0; i < Size; ++i) {
                mock_transmitted_bytes.push_back(pData[i]);
            }
        }
        if (mock_transmit_fail_count > 0) {
            mock_transmit_fail_count--;
            return HAL_ERROR;
        }
        return (HAL_StatusTypeDef)mock_transmit_status;
    }

    HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
        mock_receive_call_count++;
        mock_rx_dest = pData;
        mock_rx_size = Size;
        if (pData && Size > 0 && Size <= 16) {
            memcpy(pData, mock_rx_source, Size);
        }
        return (HAL_StatusTypeDef)mock_receive_status;
    }

    void HAL_Delay(uint32_t Delay) {
        mock_delay_call_count++;
    }
}

#include "../RocketDrivers/adc_max11614_i2c/adc_max11614_i2c.h"
#include "../RocketDrivers/adc_max11614_i2c/adc_max11614_i2c.cc"
#include "../Core/Inc/gse_types.h"

class AdcMax11614Test : public ::testing::Test {
protected:
    I2C_HandleTypeDef hi2c;
    GPIO_TypeDef port;

    void SetUp() override {
        mock_transmit_status = HAL_OK;
        mock_transmit_fail_count = 0;
        mock_transmit_call_count = 0;
        mock_transmitted_bytes.clear();
        mock_receive_status = HAL_OK;
        mock_receive_call_count = 0;
        mock_delay_call_count = 0;
        memset(mock_rx_source, 0, sizeof(mock_rx_source));
    }
};

TEST_F(AdcMax11614Test, DefaultDataStructureInitialization) {
    AdcMax11614i2c::Data data;
    EXPECT_EQ(data.channelOutput0, 0);
    EXPECT_EQ(data.channelOutput1, 0);
    EXPECT_EQ(data.channelOutput2, 0);
    EXPECT_EQ(data.channelOutput3, 0);
    EXPECT_EQ(data.channelOutput4, 0);
    EXPECT_EQ(data.channelOutput5, 0);
    EXPECT_EQ(data.channelOutput6, 0);
    EXPECT_EQ(data.channelOutput7, 0);
    EXPECT_EQ(data.channelValidBitmask, 0);
}

TEST_F(AdcMax11614Test, InitFailureStoresStateAndZeroesChannels) {
    AdcMax11614i2c adc(&hi2c, &port, 1, &port, 2);
    AdcMax11614i2c::Config config;
    config.maxAttempts = 3;

    // Make all transmits fail
    mock_transmit_status = HAL_ERROR;

    bool initResult = adc.Init(config);
    EXPECT_FALSE(initResult);
    EXPECT_TRUE(adc.HasInitFailed());

    // Check all channels are 0 and bitmask is 0
    const auto& data = adc.GetData();
    EXPECT_EQ(data.channelOutput0, 0);
    EXPECT_EQ(data.channelOutput1, 0);
    EXPECT_EQ(data.channelOutput2, 0);
    EXPECT_EQ(data.channelOutput3, 0);
    EXPECT_EQ(data.channelOutput4, 0);
    EXPECT_EQ(data.channelOutput5, 0);
    EXPECT_EQ(data.channelOutput6, 0);
    EXPECT_EQ(data.channelOutput7, 0);
    EXPECT_EQ(data.channelValidBitmask, 0);

    // StartReadAsync should return false early when init failed
    mock_receive_call_count = 0;
    bool readResult = adc.StartReadAsync();
    EXPECT_FALSE(readResult);
    EXPECT_EQ(mock_receive_call_count, 0); // HAL_I2C_Master_Receive_IT should not be called
}

TEST_F(AdcMax11614Test, InitSuccessSetsStateAndAllowsRead) {
    AdcMax11614i2c adc(&hi2c, &port, 1, &port, 2);
    AdcMax11614i2c::Config config;
    config.maxAttempts = 3;

    mock_transmit_status = HAL_OK;
    bool initResult = adc.Init(config);
    EXPECT_TRUE(initResult);
    EXPECT_FALSE(adc.HasInitFailed());

    // Both setup byte and configuration byte transmitted
    EXPECT_EQ(mock_transmit_call_count, 2);

    // StartReadAsync should proceed
    bool readResult = adc.StartReadAsync();
    EXPECT_TRUE(readResult);
    EXPECT_EQ(mock_receive_call_count, 1);
}

TEST_F(AdcMax11614Test, FetchDataBitmaskValidation) {
    AdcMax11614i2c adc(&hi2c, &port, 1, &port, 2);
    AdcMax11614i2c::Config config;
    adc.Init(config);

    // Prepare simulated RX buffer for 8 channels
    // Channel 0: valid, raw value = 1000 (0x03E8) -> byte1 = 0xF3, byte2 = 0xE8
    mock_rx_source[0] = 0xF3;
    mock_rx_source[1] = 0xE8;

    // Channel 1: corrupt header (0x00 != 0xF0) -> byte1 = 0x00, byte2 = 0x50
    mock_rx_source[2] = 0x00;
    mock_rx_source[3] = 0x50;

    // Channel 2: valid, raw value = 4095 (0x0FFF) -> byte1 = 0xFF, byte2 = 0xFF
    mock_rx_source[4] = 0xFF;
    mock_rx_source[5] = 0xFF;

    // Channels 3-7: all corrupt (0x00)
    for (int i = 6; i < 16; ++i) {
        mock_rx_source[i] = 0x00;
    }

    adc.StartReadAsync();

    // Simulate completion callback
    AdcMax11614i2c::HAL_RxCpltCallback(&hi2c);
    EXPECT_TRUE(adc.IsDataReady());

    auto optData = adc.FetchData();
    ASSERT_TRUE(optData.has_value());

    const auto& data = optData.value();
    // Valid bitmask should have bits 0 and 2 set (0b00000101 = 0x05)
    EXPECT_EQ(data.channelValidBitmask, 0b00000101);
    EXPECT_EQ(data.channelOutput0, 1000);
    EXPECT_EQ(data.channelOutput1, 0); // corrupt channel defaults to 0
    EXPECT_EQ(data.channelOutput2, 4095);
    EXPECT_EQ(data.channelOutput3, 0);
    EXPECT_EQ(data.channelOutput4, 0);
    EXPECT_EQ(data.channelOutput5, 0);
    EXPECT_EQ(data.channelOutput6, 0);
    EXPECT_EQ(data.channelOutput7, 0);

    // Stored data should match
    EXPECT_EQ(adc.GetData().channelValidBitmask, 0b00000101);
}

TEST_F(AdcMax11614Test, FetchDataAllChannelsValid) {
    AdcMax11614i2c adc(&hi2c, &port, 1, &port, 2);
    AdcMax11614i2c::Config config;
    adc.Init(config);

    for (int i = 0; i < 8; ++i) {
        mock_rx_source[i * 2] = 0xF0 | ((i + 1) & 0x0F);
        mock_rx_source[i * 2 + 1] = 0x20;
    }

    adc.StartReadAsync();
    AdcMax11614i2c::HAL_RxCpltCallback(&hi2c);

    auto optData = adc.FetchData();
    ASSERT_TRUE(optData.has_value());

    const auto& data = optData.value();
    EXPECT_EQ(data.channelValidBitmask, 0xFF);
    EXPECT_EQ(data.channelOutput0, (1 << 8) | 0x20);
    EXPECT_EQ(data.channelOutput7, (8 << 8) | 0x20);
}

TEST_F(AdcMax11614Test, GseDataStructBitmaskField) {
    GseData gseData;
    EXPECT_EQ(gseData.externalAdcValidBitmask, 0);
    gseData.externalAdcValidBitmask = 0b00000001; // Only channel 0 valid
    EXPECT_EQ(gseData.externalAdcValidBitmask, 1);
}

