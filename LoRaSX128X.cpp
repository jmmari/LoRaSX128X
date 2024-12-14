
#include "LoRaSX128X.h"
#include <math.h>

static spi_device_handle_t __spi;
static int __send_packet_lost = 0;

#define SPI_TRANSMIT 1
#define BUFFER_IO 1
// #define LORA24_DEBUG
//----------------------------------------------------------------------------------------------------------------------------------------
LoRaSX128X::LoRaSX128X(gpio_num_t pinMiso, gpio_num_t pinMosi, gpio_num_t pinSclk, gpio_num_t pinCS, gpio_num_t pinRST, gpio_num_t apBusyPin, int8_t txPower)
    : _pinMiso(pinMiso), _pinMosi(pinMosi), _pinSclk(pinSclk), _pinCS(pinCS), _pinRST(pinRST), _pinBusy(apBusyPin), _txPower(txPower) {
        
        _implicit = HEADER_TYPE_IMPLICIT;
#ifdef LORA24_DEBUG
        esp_log_level_set("*", ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "MISO pin: %d, MOSI pin: %d, SCK pin: %d, NSS pin: %d", pinMiso, pinMosi, pinSclk, pinCS);
        ESP_LOGI(TAG, " RESET pin: %d, BUSY pin: %d", pinRST, apBusyPin);
#endif
}
//----------------------------------------------------------------------------------------------------------------------------------------
void printBufferHex(const uint8_t* buffer, size_t len) {
    if (buffer == NULL || len == 0) {        ESP_LOGW(TAG, "------------------------------------------ ! Buffer is empty or NULL.");        return;    }
    size_t hexLen = len * 2 + 1;
    char* hexString = (char*) malloc(hexLen);
    if (hexString == NULL) {        ESP_LOGE(TAG, " --------------------------------------- ! Failed to allocate memory for hex string.");        return;    }
    for (size_t i = 0; i < len; i++) {        sprintf(&hexString[i * 2], "%02X", buffer[i]);    }
    hexString[len * 2] = '\0';
    ESP_LOGW(TAG, " Buffer in HEX : %s", hexString);
    free(hexString);
}
//----------------------------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::checkPinConfiguration() {
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI("PinConfig", "Checking pin configuration...");
    int pinList[] = {_pinMosi, _pinMiso, _pinSclk, _pinCS, _pinRST, _pinBusy};  // Modify this list based on your setup
    const char* pinNames[] = {"MOSI", "MISO", "SCK", "NSS", "RESET", "DIO0", "BUSY"};
    for (int i = 0; i < sizeof(pinList) / sizeof(pinList[0]); i++) {
        gpio_num_t pin = (gpio_num_t)pinList[i];
        int pinLevel = gpio_get_level(pin);
        ESP_LOGI("PinConfig", "Pin %s (GPIO %d): Level=%d", pinNames[i], pin, pinLevel);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------
int LoRaSX128X::waitWhileBusy() {
    int timeElapsed = 0;
    while (gpio_get_level(_pinBusy) == 1) {
        vTaskDelay(pdMS_TO_TICKS(5));  // Wait for 1 ms before checking again
        timeElapsed++;
        if (timeElapsed % 200 == 0) {    ESP_LOGI(TAG, " ------------------------------> Module still busy... %d s elapsed", timeElapsed*5);   }
    }
    return 1; // Success
}
//----------------------------------------------------------------------------------------------------------------------------------------


int LoRaSX128X::full_init(const LoRaParameters& params) {
    esp_err_t ret;
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGW(TAG, "LoRa Initialize...");

    // Reset and configure GPIO pins
    gpio_reset_pin(_pinRST);
    gpio_reset_pin(_pinCS);

    ret = gpio_set_direction(_pinRST, GPIO_MODE_INPUT_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "------------------------------------------- Failed to set direction for RST pin");
    } else {
        ESP_LOGI(TAG, " -> RST pin configured as output");
    }

    ret = gpio_set_direction(_pinCS, GPIO_MODE_INPUT_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "------------------------------------------- Failed to set direction for CS pin");
    } else {
        ESP_LOGI(TAG, " -> CS pin configured as output");
    }

    gpio_pullup_dis(_pinRST);
    gpio_pulldown_dis(_pinRST);
    gpio_pullup_dis(_pinCS);
    gpio_pulldown_dis(_pinCS);

    ret = gpio_set_level(_pinRST, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "-------------------------------------------- Failed to set RST pin level");
    }

    ret = gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "-------------------------------------------- Failed to set CS pin level");
    }

    // Configure the BUSY pin
    gpio_reset_pin(_pinBusy);
    ret = gpio_set_direction(_pinBusy, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "-------------------------------------------- Failed to set direction for Busy pin");
    } else {
        ESP_LOGI(TAG, " -> Busy pin configured as input");
    }

    gpio_pullup_dis(_pinBusy);
    gpio_pulldown_dis(_pinBusy);

    checkPinConfiguration();

    // SPI Bus Initialization
    spi_bus_config_t bus = {0};
    bus.mosi_io_num = _pinMosi;
    bus.miso_io_num = _pinMiso;
    bus.sclk_io_num = _pinSclk;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;

    ret = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " -------------------------------- ! Failed to initialize SPI bus !");
        return 0;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 8 * 1000 * 1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 7;
    devcfg.flags = 0;

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &__spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "------------------------------------------ Failed to add SPI device");
        return 0;
    }

    // Reset and configure LoRa module
    reset();
    waitWhileBusy();
    setStandBy();

    setLoRaMode();
    setFrequencyInHz(params.frequencyHz);
    setBufferBaseAddr();

    setModulationParameters(params.spreadingFactor, params.bandwidth, params.codingRate);
    setPacketParameters(params.preambleLength, params.headerType, params.payloadLength, params.crcMode, params.iqMode);

    setSyncWord(params.syncWord);
    setRegulatorMode(0);
    setSaveContext();
    setStandBy();

    ESP_LOGI(TAG, "SX1280 initialized successfully");
    return 1; // Success
}

//***********************************************************************************************************************************
void LoRaSX128X::sendCommandCode(const uint8_t *cmd, size_t cmdLength) {
    waitWhileBusy();
    gpio_set_level(_pinCS, 0); // Pull CS low
    spi_transaction_t t = {
        .length = cmdLength * 8,
        .rxlength = 0,
        .tx_buffer = cmd,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1); // Pull CS high
    if (ret != ESP_OK) {        ESP_LOGE(TAG, "------------------------------------------- SPI transaction failed: %s", esp_err_to_name(ret));    }
    waitWhileBusy();
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setStandBy() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set standby with command value = 0x%02X, and mode = 0x%02X.", CMD_SET_STANDBY, MODE_STDBY_RC);
#endif
    uint8_t setStandbyCmd[] = { CMD_SET_STANDBY, MODE_STDBY_RC };
    sendCommandCode(setStandbyCmd, sizeof(setStandbyCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setRx(uint16_t timeOut) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set RX with command value = 0x%02X.", CMD_SET_RX);   
#endif
    setBufferBaseAddr();
    setPacketParameters(currentPreambleLength, currentHeaderType, 0xFF, currentCrcType, currentIQInvertStatus);
    gpio_set_level(GPIO_NUM_21, 1);
    gpio_set_level(GPIO_NUM_10, 0); 
    uint8_t setRxCmd[] = { CMD_SET_RX, PERIODBASE_01_MS, (uint8_t)((timeOut >> 8) & 0xFF), (uint8_t)(timeOut & 0xFF) };
    sendCommandCode(setRxCmd, sizeof(setRxCmd));
    vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms delay
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setTx() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set TX with command value = 0x%02X.", CMD_SET_TX);
#endif
    gpio_set_level(GPIO_NUM_21, 0);
    gpio_set_level(GPIO_NUM_10, 1);
    uint8_t setTxCmd[] = { CMD_SET_TX, PERIODBASE_01_MS, 0x00, 0x00 };
    sendCommandCode(setTxCmd, sizeof(setTxCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setSaveContext() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set Save Context with command value = 0x%02X.", CMD_SET_SAVE_CONTEXT);
#endif
    uint8_t setSaveContextCmd[] = { CMD_SET_SAVE_CONTEXT };
    sendCommandCode(setSaveContextCmd, sizeof(setSaveContextCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setLoRaMode() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set Packet Type to LoRa TX with command value = 0x%02X, and parameter 0x%02X.", CMD_SET_PACKET_TYPE, PACKET_TYPE_LORA);
#endif
    uint8_t setPacketTypeCmd[] = { CMD_SET_PACKET_TYPE, PACKET_TYPE_LORA }; 
    sendCommandCode(setPacketTypeCmd, sizeof(setPacketTypeCmd));
}
void LoRaSX128X::setRegulatorMode(uint8_t mode) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set Regulator mode with command value = 0x%02X, and parameter 0x%02X.", CMD_SET_REGULATOR_MODE, mode);
#endif
    uint8_t setRegModeCmd[] = { CMD_SET_REGULATOR_MODE, mode }; 
    sendCommandCode(setRegModeCmd, sizeof(setRegModeCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setFrequencyInHz(uint32_t rfFrequency) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    uint32_t freq = static_cast<uint32_t>((double)rfFrequency / (double)FREQ_STEP);
    uint8_t setRfFrequencyCmd[] = {
        CMD_SET_RF_FREQUENCY, 
        static_cast<uint8_t>((freq >> 16) & 0xFF),
        static_cast<uint8_t>((freq >> 8) & 0xFF),
        static_cast<uint8_t>((freq) & 0xFF),
    };
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Setting frequency with command value = 0x%02X, and parameter 0x%02X-0x%02X-0x%02X", (unsigned int)CMD_SET_RF_FREQUENCY, (unsigned int)setRfFrequencyCmd[1], (unsigned int)setRfFrequencyCmd[2], (unsigned int)setRfFrequencyCmd[3]);
#endif
    sendCommandCode(setRfFrequencyCmd, sizeof(setRfFrequencyCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setBufferBaseAddr() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "Set Buffer Base Address with command value = 0x%02X.", CMD_SET_BUFFER_BASE_ADDRESS);
#endif
    uint8_t setBufferBaseAddrCmd[] = { CMD_SET_BUFFER_BASE_ADDRESS, 0x00, 0x00 }; 
    sendCommandCode(setBufferBaseAddrCmd, sizeof(setBufferBaseAddrCmd));
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setModulationParameters(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "______________________________________________________________________________________________________________________");
    ESP_LOGI(TAG, "Set Modulation Parameters with command value = 0x%02X, spreadingFactor= 0x%02X, bandwidth= 0x%02X, codingRate= 0x%02X.", CMD_SET_MODULATION_PARAMS, spreadingFactor, bandwidth, codingRate);
#endif
    uint8_t setModulationParamsCmd[] = {
        CMD_SET_MODULATION_PARAMS, 
        spreadingFactor,
        bandwidth,
        codingRate
    };
    sendCommandCode(setModulationParamsCmd, sizeof(setModulationParamsCmd));
    switch(spreadingFactor){
        case LORA_SF5:
        case LORA_SF6:
#ifdef LORA24_DEBUG
            ESP_LOGW(TAG, "setModulationParameters: writing 0x1E.");
#endif
            writeByteToRegister(0x0925, 0x1E);            break;
        case LORA_SF7:
        case LORA_SF8:
#ifdef LORA24_DEBUG
            ESP_LOGW(TAG, "setModulationParameters: writing 0x37.");
#endif
            writeByteToRegister(0x0925, 0x37);            break;
        default:
#ifdef LORA24_DEBUG
            ESP_LOGW(TAG, "setModulationParameters: writing 0x32.");
#endif
            writeByteToRegister(0x0925, 0x32);            break;        
    }
    writeByteToRegister(0x093C, 0x1);
}
//-----------------------------------------------------------------------------------------------------------------
uint8_t buildPacketParam1(uint8_t preambleLength) {
    // Calculate exponent and mantissa
    uint8_t e = 1;
    uint8_t m = 1;
    uint32_t len = 0;
    for (; e <= 15; e++) {
        for (m = 1; m <= 15; m++) {
            len = m * (uint32_t(1) << e);
            if (len >= preambleLength) {
                break;
            }
        }
        if (len >= preambleLength) {
            break;
        }
    }
    uint8_t packetParam1 = (e << 4) | m;
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Preamble : Mantissa 0x%02X ; Exponent 0x%02X ; param = 0x%02X", m, e, packetParam1);
#endif
    return packetParam1;
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setPacketParameters(uint8_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcMode, uint8_t invertIQ) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "_________________________________________________________________________________________________________________________________________________________________");
    ESP_LOGI(TAG, "Set Packet Parameters with command value = 0x%02X, preambleLength= 0x%02X (0x%02X), headerType= 0x%02X, payloadLength= 0x%02X, crcMode= 0x%02X, invertIQ= 0x%02X.", CMD_SET_PACKET_PARAMS, preambleLength, buildPacketParam1(preambleLength), headerType, payloadLength, crcMode, invertIQ);
#endif
    uint8_t setPacketParamsCmd[] = {
        CMD_SET_PACKET_PARAMS,
        buildPacketParam1(preambleLength), 
        headerType,
        payloadLength,
        crcMode,
        invertIQ,
        0x00,
        0x00
    };
    sendCommandCode(setPacketParamsCmd, sizeof(setPacketParamsCmd));
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Payload Length : 0x%02X", readByteRegister(0x901));
#endif
}
//-----------------------------------------------------------------------------------------------------------------
void LoRaSX128X::setTxParameters(int8_t power, uint8_t rampTime) {
    // int8_t power = 10; // TX power between -18 to +13 dBm
    // uint8_t rampTime = 0xE0; // 20 us ramp time
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "______________________________________________________________________________________");
    ESP_LOGI(TAG, "Set TX Parameters with command value = 0x%02X, power= 0x%02X, and rampTime=0x%02X.", CMD_SET_TX_PARAMS, power, rampTime);
#endif
    uint8_t setTxParamsCmd[] = {
        CMD_SET_TX_PARAMS, // Command: SetTxParams
        (uint8_t) power,
        rampTime
    };
    sendCommandCode(setTxParamsCmd, sizeof(setTxParamsCmd));
}
//-----------------------------------------------------------------------------------------
void LoRaSX128X::clear_irq() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    uint8_t clrIrqStatusCmd[] = {
        CMD_CLEAR_IRQ_STATUS,
        (uint8_t)(0xFF),
        (uint8_t)(0xFF)
    };
    sendCommandCode(clrIrqStatusCmd, sizeof(clrIrqStatusCmd));
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "IRQ flags cleared successfully");
#endif
}
//-----------------------------------------------------------------------------------------------------------
void LoRaSX128X::set_dio_mapping(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) { 
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    uint8_t command[] = {
        CMD_SET_DIO_IRQ_PARAMS,
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask & 0xFF),
        (uint8_t)(dio1Mask >> 8),
        (uint8_t)(dio1Mask & 0xFF),
        (uint8_t)(dio2Mask >> 8),
        (uint8_t)(dio2Mask & 0xFF),
        (uint8_t)(dio3Mask >> 8),
        (uint8_t)(dio3Mask & 0xFF)
    };
    sendCommandCode(command, sizeof(command));   
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "DIO IRQ params set successfully");
#endif
}
//----------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::readChipRegisters() {
    // Example of reading specific registers
    uint16_t reg15_3 = readWordRegister(0x0153); // Firmware Version
    uint8_t reg89_5 = readByteRegister(0x0895); // Manual Gain
    uint8_t reg89_E = readByteRegister(0x089E); // LNA Gain Value
    uint8_t reg89_F = readByteRegister(0x089F); // LNA Gain Control
    uint8_t reg90_1 = readByteRegister(0x0901); // Payload Length
    uint8_t reg90_3 = readByteRegister(0x0903); // LoRa Header Mode
    uint8_t reg92_5 = readByteRegister(0x0925); // Spreading Factor
    uint8_t reg93_C = readByteRegister(0x093C); // Frequency Error Correction
    uint8_t reg94_2 = readByteRegister(0x0942); // CAD Detection Peak
    uint8_t reg95_4 = readByteRegister(0x0954); // Header CRC
    uint8_t reg95_0 = readByteRegister(0x0950); // Coding Rate
    uint8_t reg95_5 = readByteRegister(0x0955); // FEI Byte 1 (M)
    uint8_t reg95_6 = readByteRegister(0x0956); // FEI Byte 0 (L)

    ESP_LOGI(TAG, " ________________________________________________________");
    // ESP_LOGI(TAG, "Read Register 0x0153: 0x%04X", reg15_3);
    ESP_LOGI(TAG, " - Firmware Version           : 0x%04X", ((reg15_3 << 8) & 0xFF00) | ((reg15_3 >> 8) & 0x00FF)); // Example
    // ESP_LOGI(TAG, "Read Register 0x0895: 0x%02X", reg89_5);
    ESP_LOGI(TAG, " - Manual Gain                : 0x%02X", reg89_5);
    // ESP_LOGI(TAG, "Read Register 0x089E: 0x%02X", reg89_E);
    ESP_LOGI(TAG, " - LNA Gain Value             : 0x%02X", reg89_E);
    // ESP_LOGI(TAG, "Read Register 0x089F: 0x%02X", reg89_F);
    ESP_LOGI(TAG, " - LNA Gain Control           : 0x%02X", reg89_F);
    // ESP_LOGI(TAG, "Read Register 0x0901: 0x%02X", reg90_1);
    ESP_LOGI(TAG, " - Payload Length             : 0x%02X", reg90_1);
    // ESP_LOGI(TAG, "Read Register 0x0903: 0x%02X", reg90_3);
    ESP_LOGI(TAG, " - LoRa Header Mode           : %s", (reg90_3 == 0xC2) ? "EXPLICIT" : "IMPLICIT");
    // ESP_LOGI(TAG, "Read Register 0x0925: 0x%02X", reg92_5);
    ESP_LOGI(TAG, " - Spreading Factor           : 0x%02X", reg92_5);
    // ESP_LOGI(TAG, "Read Register 0x093C: 0x%02X", reg93_C);
    ESP_LOGI(TAG, " - Frequency Error Correction : 0x%02X", reg93_C);
    // ESP_LOGI(TAG, "Read Register 0x0942: 0x%02X", reg94_2);
    ESP_LOGI(TAG, " - CAD Detection Peak         : 0x%02X", reg94_2);
    // ESP_LOGI(TAG, "Read Register 0x0954: 0x%02X", reg95_4);
    ESP_LOGI(TAG, " - Header CRC                 : %s", (reg95_4 == 0x00) ? "OFF" : "ON");
    // ESP_LOGI(TAG, "Read Register 0x0950: 0x%02X", reg95_0);
    ESP_LOGI(TAG, " - Received Coding Rate       : 0x%02X", reg95_0);
    // ESP_LOGI(TAG, "Read Register 0x0954: 0x%02X", reg95_5);
    ESP_LOGI(TAG, " - FEI Byte 2 (H)             : 0x%02X", reg95_5);
    // ESP_LOGI(TAG, "Read Register 0x0955: 0x%02X", reg95_6);
    ESP_LOGI(TAG, " - FEI Byte 1 (M)             : 0x%02X", reg95_6);
    // ESP_LOGI(TAG, "Read Register 0x0956: 0x%02X", reg95_6);
    ESP_LOGI(TAG, " - FEI Byte 0 (L)             : 0x%02X", reg95_6);
}
//----------------------------------------------------------------------------------------------------------------------
uint8_t LoRaSX128X::setSyncWord(uint8_t syncWord) {
    uint8_t upperNibble = (syncWord & 0xF0) >> 4; // X
    uint8_t lowerNibble = (syncWord & 0x0F);      // Y
    uint8_t currentRegValue;
    uint8_t newRegValue;
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, " ____________________ Set sync word 0x%02X ____________________", syncWord);
    ESP_LOGI(TAG, "Writing sync word 0x%02X : Upper: 0x%02X ; Lower : 0x%02X", syncWord, upperNibble, lowerNibble);
#endif
    currentRegValue = readByteRegister(REG_LORA_SYNC_WORD_MSB);
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Setting sync word MSB: read 0x%02X at register 0x%04X", currentRegValue, REG_LORA_SYNC_WORD_MSB);
#endif
    newRegValue = (upperNibble << 4) | (currentRegValue & 0x0F);
    writeByteToRegister(REG_LORA_SYNC_WORD_MSB, newRegValue);
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Wrote sync word MSB: 0x%02X to register 0x%04X", newRegValue, REG_LORA_SYNC_WORD_MSB);
    printStatus();
#endif    
    currentRegValue = readByteRegister(REG_LORA_SYNC_WORD_LSB);
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Setting sync word LSB: read 0x%02X at register 0x%04X", currentRegValue, REG_LORA_SYNC_WORD_MSB);
#endif
    newRegValue = (lowerNibble << 4) | (currentRegValue & 0x0F);
    writeByteToRegister(REG_LORA_SYNC_WORD_LSB, newRegValue);
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Wrote sync word LSB: 0x%02X to register 0x%04X", newRegValue, REG_LORA_SYNC_WORD_LSB);
    printStatus();
#endif
    uint8_t syncWordRead = readSyncWord(); //  ((msb & 0xF0)) | ((lsb & 0xF0) >> 4);
#ifdef LORA24_DEBUG
    ESP_LOGI(TAG, "Read Sync Word: 0x%02X", syncWordRead);
    ESP_LOGI(TAG, " _________________ Sync word set to 0x%02X ______________________", syncWord);
#endif
    return syncWordRead;
}
//----------------------------------------------------------------------------------------------------------------------
uint8_t LoRaSX128X::readSyncWord() {
    uint8_t msb = readByteRegister(REG_LORA_SYNC_WORD_MSB);
    uint8_t lsb = readByteRegister(REG_LORA_SYNC_WORD_LSB);
    uint8_t syncWordRead = ((msb & 0xF0)) | ((lsb & 0xF0) >> 4);
    return syncWordRead;
}
//----------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::testSyncWordSetting(uint8_t desiredSyncWord) {
    uint8_t syncWordRead = readSyncWord(); //  ((msb & 0xF0)) | ((lsb & 0xF0) >> 4);
    ESP_LOGI(TAG, "Read Sync Word: 0x%02X", syncWordRead);
    if (syncWordRead == desiredSyncWord) {
        ESP_LOGI(TAG, "Sync Word successfully set to 0x%02X", syncWordRead);
    } else {
        ESP_LOGE(TAG, "Sync Word mismatch! Expected: 0x%02X, Got: 0x%02X", desiredSyncWord, syncWordRead);
    }
}
//----------------------------------------------------------------------------------------- esp_err_t
uint8_t LoRaSX128X::readByteRegister(uint16_t reg) {
    uint8_t txData[5] = { CMD_READ_REGISTER, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), 0x00, 0x00 };
    uint8_t rxData[5] = { 0 };
    rxData[0] = 0;rxData[1] = 0;rxData[2] = 0;rxData[3] = 0;rxData[4] = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(txData);
    t.tx_buffer = txData;
    t.rx_buffer = rxData;

    gpio_set_level(_pinCS, 0);
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI readRegister transaction failed: %s", esp_err_to_name(ret));
        return 0x00; 
    }
    // ESP_LOGI(TAG, "Read Register 0x%04X: 0x%02X", reg, rxData[sizeof(txData)-1]);
    return rxData[sizeof(txData)-1];
}
//----------------------------------------------------------------------------------------------------------------------
uint16_t LoRaSX128X::readWordRegister(uint16_t reg) {
    uint8_t txData[6] = { CMD_READ_REGISTER, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), 0x00, 0x00, 0x00 };
    uint8_t rxData[6] = { 0 };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(txData); 
    t.tx_buffer = txData;
    t.rx_buffer = rxData;

    gpio_set_level(_pinCS, 0);
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI readRegister transaction failed: %s", esp_err_to_name(ret));
        return 0x0000; // Return an invalid value or handle as needed
    }
    // ESP_LOGI(TAG, "Read Register 0x%04X: 0x%02X", reg, rxData[sizeof(txData)-1]);
    uint16_t regVal = ((uint16_t)rxData[sizeof(txData)-2] << 8) | (uint16_t)rxData[sizeof(txData)-1];
    return regVal;
}
//----------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::writeRegister(uint16_t reg, uint8_t* data, size_t len) {
    uint8_t out[260] = { 0x00 };
    out[0] = CMD_WRITE_REGISTER;
    out[1] = (reg >> 8) & 0xFF;
    out[2] = reg & 0xFF;       
    memcpy(&out[3], data, len);  
#ifdef LORA24_DEBUG  
    printBufferHex(out, 3 + len);
#endif
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
    spi_transaction_t t = {
        .length = 8 * (len + 3),
        .rxlength = 0,
        .tx_buffer = out,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI writeRegister transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else { ESP_LOGI(TAG, "SPI writeRegister transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
}
//----------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::writeByteToRegister(uint16_t reg, uint8_t data) {
    uint8_t out[4] = { 0x00 };
    out[0] = CMD_WRITE_REGISTER;
    out[1] = (reg >> 8) & 0xFF;
    out[2] = reg & 0xFF;   
    out[3] = data;
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
    spi_transaction_t t = {
        .length = 8*sizeof(out),
        .rxlength = 0, 
        .tx_buffer = out,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI writeRegister transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else { ESP_LOGI(TAG, "SPI writeRegister transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
}
//----------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::writeWordToRegister(uint16_t reg, uint16_t data) {
    uint8_t out[5] = { 0x00 };
    out[0] = CMD_WRITE_REGISTER;
    out[1] = (reg >> 8) & 0xFF;
    out[2] = reg & 0xFF;   
    out[3] = (data >> 8) & 0xFF;
    out[4] = data & 0xFF;
    waitWhileBusy();
    gpio_set_level(_pinCS, 0); 
    spi_transaction_t t = {
        .length = 8*sizeof(out),
        .rxlength = 0,
        .tx_buffer = out, 
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(__spi, &t);
    gpio_set_level(_pinCS, 1); 
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI writeRegister transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else { ESP_LOGI(TAG, "SPI writeRegister transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
}
//----------------------------------------------------------------------------------------- esp_err_t
void LoRaSX128X::send_packet(uint8_t *buf, size_t size) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    waitWhileBusy();
    int8_t power = 0; 
    uint8_t rampTime = RADIO_RAMP_20_US; 
    setTxParameters(power, rampTime);
    setBufferBaseAddr();
    write_buffer(REG_FIFO, buf, size);
    setPacketParameters(currentPreambleLength, currentHeaderType, size, currentCrcType, currentIQInvertStatus);
    setTx();
    ESP_LOGI(TAG, " --------------------------------------------------------------- Packet TX ! ");
}
//----------------------------------------------------------------------------------------- esp_err_t
void LoRaSX128X::sendPacket(uint8_t *buf, size_t size) {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    waitWhileBusy();
    int8_t power = 0; 
    uint8_t rampTime = RADIO_RAMP_20_US; 
    setTxParameters(power, rampTime);
    setBufferBaseAddr();

    /*uint8_t out[255];
    for (uint8_t i = 0; i < 255; i++) {
        out[i] = i;  // Fill buffer with 0x0 to 0xF
    }*/
    clear_irq();
    set_dio_mapping(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, 0x00, 0x00);
    write_buffer(REG_FIFO, buf, size);
    setPacketParameters(currentPreambleLength, currentHeaderType, size, currentCrcType, currentIQInvertStatus);
    setTx();
    // ESP_LOGI(TAG, " -------------------------------------------------------------------------- Packet transmit in progress... ");
    // vTaskDelay(pdMS_TO_TICKS(500));
    uint16_t irqStatus = 0;
    while (!(irqStatus & IRQ_TX_DONE)) {
        // Read the IRQ status
        irqStatus = getIrq();
        // Optionally log or handle other IRQs (e.g., timeout)
        if (irqStatus & IRQ_RX_TX_TIMEOUT) {
            ESP_LOGE(TAG, "------------------------------------------------------------------------------ TX timed out!");
            break;
        }
        ESP_LOGI(TAG, " Waiting for packet transmit to finish... ");
        // Delay to prevent flooding the IRQ check
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, " --------------------------------------------------------------- Packet sent ! ");
}
//*******************************************************************************************************************************
uint8_t LoRaSX128X::printStatus() {
    uint8_t status = getStatus();
    // ESP_LOGW(TAG, " GetStatus : Read Status Byte: 0x%02X\n", status);
    interpretStatus(status);
    return status;
}
//----------------------------------------------------------------------------------
uint8_t LoRaSX128X::getStatus() {
    uint8_t txData[1] = {CMD_GET_STATUS};
    uint8_t rxData[1] = { 0x00 };
    waitWhileBusy();
    esp_err_t gpioErr = gpio_set_level(_pinCS, 0);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "------------------------------ gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(txData);
    t.tx_buffer = txData;
    t.rx_buffer = rxData;

#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI getStatus transaction failed: %s", esp_err_to_name(ret)); }
    gpioErr = gpio_set_level(_pinCS, 1);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "--------------------------- gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    waitWhileBusy();
    return rxData[0];
}
//----------------------------------------------------------------------------------
void LoRaSX128X::interpretStatus(uint8_t status) {
    
    ESP_LOGW("LoRaSX128X", " --- Chip Status : 0x%02X ---", status);
    // Chip Busy (Bit 0)
    bool isBusy = (status & 0x01);
    ESP_LOGW("LoRaSX128X", "Chip Busy: %s", isBusy ? "Yes" : "No");
    // Reserved Bit (Bit 1) - Ignored as per datasheet
    // Command Status (Bits 4:2)
    uint8_t cmdStatus = (status >> 2) & 0x07;
    const char* cmdStatusStr = NULL;
    switch (cmdStatus) {
        case 0x0: cmdStatusStr = "Reserved"; break;
        case 0x1: cmdStatusStr = "Command Successful"; break;
        case 0x2: cmdStatusStr = "Data Available to Host"; break;
        case 0x3: cmdStatusStr = "Command Timeout"; break;
        case 0x4: cmdStatusStr = "Command Processing Error"; break;
        case 0x5: cmdStatusStr = " -------------------------------> ! Failure to Execute Command !"; break;
        case 0x6: cmdStatusStr = "Command Tx Done"; break;
        default:  cmdStatusStr = "Unknown Command Status"; break;
    }
    ESP_LOGW("LoRaSX128X", "Command Status: %s", cmdStatusStr);

    // Circuit Mode (Bits 7:5)
    uint8_t mode = (status >> 5) & 0x07;
    const char* modeStr = NULL;
    switch (mode) {
        case 0x0: modeStr = "Reserved"; break;
        case 0x1: modeStr = "Reserved"; break;
        case 0x2: modeStr = "Standby RC"; break;
        case 0x3: modeStr = "Standby XOSC"; break;
        case 0x4: modeStr = "Frequency Synthesis"; break;
        case 0x5: modeStr = "Receive"; break;
        case 0x6: modeStr = "Transmit"; break;
        default:  modeStr = "Unknown Mode"; break;
    }
    ESP_LOGW("LoRaSX128X", "Operating Mode: %s", modeStr);
    // ESP_LOGW("LoRaSX128X", " -------------------");
}
//----------------------------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::write_buffer(uint8_t startAddr, uint8_t *val, size_t len) {
    uint8_t out[260] = { 0 }; 
    out[0] = CMD_WRITE_BUFFER;  
    out[1] = startAddr;
    out[259] = 0;
    memcpy(&out[2], val, len);

    // printBufferHex(out, 2 + len);
    waitWhileBusy();
    
    spi_transaction_t t = {
        .length = (size_t)(8 * (2 + len)), 
        .rxlength = 0,    
        .tx_buffer = out, 
        .rx_buffer = NULL 
    };
    gpio_set_level(_pinCS, 0);
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "SPI sending buffer starts...");
#endif
    esp_err_t ret;
#if SPI_TRANSMIT
    ret = spi_device_transmit(__spi, &t);
#else
    ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    
    if (ret != ESP_OK) { ESP_LOGE(TAG, "------------------------------------------- SPI write_buffer transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else { ESP_LOGW(TAG, "SPI write_buffer transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
}
//----------------------------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::read_buffer(uint8_t offset, uint8_t *buffer, size_t size) {
    uint8_t rxData[260] = { 0 };
    uint8_t out[260] = { CMD_READ_BUFFER, offset, 0x00 };
    spi_transaction_t t = {
        .length = 8 * (size + 3),
        .rxlength = 8 * (size + 3),
        .tx_buffer = out,
        .rx_buffer = rxData
    };
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "------------------------------------------ SPI read_reg_buffer transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else{ ESP_LOGE(TAG, "SPI read_reg_buffer transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    memcpy(buffer, &rxData[3], size);
    waitWhileBusy();
}
//----------------------------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::reset(void){
    vTaskDelay(pdMS_TO_TICKS(10));    gpio_set_level((gpio_num_t)_pinRST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));    gpio_set_level((gpio_num_t)_pinRST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}
//----------------------------------------------------------------------------------------------------------------------------------------
uint8_t LoRaSX128X::getPacketType() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
    ESP_LOGI(TAG, "getPacketType()");
#endif
    uint8_t command[3] = { CMD_GET_PACKET_TYPE, 0x00, 0x00 };
    uint8_t answer[3] = { 0x05 };
    spi_transaction_t t = {
        .length = sizeof(command) * 8,
        .rxlength = sizeof(answer) * 8,
        .tx_buffer = command,
        .rx_buffer = answer
    };
    waitWhileBusy();  
    gpio_set_level(_pinCS, 0);
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "---------------------------------------- Failed to get packet type: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG  
    else{  
        ESP_LOGE(TAG, "SPI getPacketType transaction succeeded: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "Received bytes: %02X %02X %02X", answer[0], answer[1], answer[2]);    
    }
#endif
    waitWhileBusy();  
        return answer[2];
}
//----------------------------------------------------------------------------------
RxBufferStatus LoRaSX128X::getRxBufferStatus() {
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "________________________________________________________________");
#endif
    uint8_t txData[4] = { CMD_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00 };
    uint8_t rxData[4] = { 0x00 };
    waitWhileBusy();
    esp_err_t gpioErr = gpio_set_level(_pinCS, 0);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "------------------------------ gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(txData);
    t.rxlength = 8 * sizeof(rxData);
    t.tx_buffer = txData;
    t.rx_buffer = rxData;
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "----------------------------------- > SPI getStatus transaction failed: %s", esp_err_to_name(ret));
    }
    gpioErr = gpio_set_level(_pinCS, 1);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "--------------------------- gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    RxBufferStatus rbs = { rxData[2], rxData[3] } ;
    waitWhileBusy();
    return rbs;
}
//----------------------------------------------------------------------------------
RxPacketStatus LoRaSX128X::getPacketStatus() {
    uint8_t txData[7] = { CMD_GET_PACKET_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rxData[7] = { 0x00 };
    waitWhileBusy();
    esp_err_t gpioErr = gpio_set_level(_pinCS, 0);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "------------------------------ gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(txData);
    t.rxlength = 8 * sizeof(rxData);
    t.tx_buffer = txData;
    t.rx_buffer = rxData;

#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "----------------------------------- > SPI getStatus transaction failed: %s", esp_err_to_name(ret));
    }
    gpioErr = gpio_set_level(_pinCS, 1);
    if (gpioErr != ESP_OK) {        ESP_LOGE(TAG, "--------------------------- gpio_set_level failed: %s", esp_err_to_name(gpioErr));    }
    RxPacketStatus rps = { rxData[2], rxData[3] } ;
    waitWhileBusy();
    return rps;
}
//-------------------------------------------------------------------------------------------
uint16_t LoRaSX128X::getIrq(void) {
    uint8_t cmdGetIrqStatus[4] = { CMD_GET_IRQ_STATUS, 0x00, 0x00, 0x00 };
    uint8_t irqResponse[4] = { 0 };  // To store the response

    spi_transaction_t t = {
        .length = sizeof(cmdGetIrqStatus) * 8,
        .rxlength = sizeof(irqResponse) * 8,
        .tx_buffer = cmdGetIrqStatus,        
        .rx_buffer = irqResponse
    };
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
 #if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI get_irq transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG
    else{ ESP_LOGE(TAG, "SPI get_irq transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
    uint16_t irqStatus = (irqResponse[2] << 8) | irqResponse[3];
    return irqStatus;
}
//----------------------------------------------------------------------------------------------------------------------------------------
void LoRaSX128X::printIRQStatus(uint16_t irqStatus) {
    ESP_LOGI(TAG, "IRQ Status: 0x%04X", irqStatus);
    // Bit 0: TxDone
    if (irqStatus & (1 << 0)) {        ESP_LOGI(TAG, "[IRQ] TxDone: Transmission complete");     }
        // Bit 1: RxDone
    if (irqStatus & (1 << 1)) {        ESP_LOGI(TAG, "[IRQ] RxDone: Reception complete");        }
    // Bit 2: SyncWordValid
    if (irqStatus & (1 << 2)) {        ESP_LOGI(TAG, "[IRQ] SyncWordValid: Sync word detected");    }
    // Bit 3: SyncWordError
    if (irqStatus & (1 << 3)) {        ESP_LOGI(TAG, "[IRQ] SyncWordError: Sync word error");    }
    // Bit 4: HeaderValid
    if (irqStatus & (1 << 4)) {        ESP_LOGI(TAG, "[IRQ] HeaderValid: Valid header received");    }
    // Bit 5: HeaderError
    if (irqStatus & (1 << 5)) {        ESP_LOGI(TAG, "[IRQ] HeaderError: Header error");         }
    // Bit 6: CrcError
    if (irqStatus & (1 << 6)) {        ESP_LOGI(TAG, "[IRQ] CrcError: CRC error");               }
    // Bit 7: RangingSlaveResponseDone
    if (irqStatus & (1 << 7)) {        ESP_LOGI(TAG, "[IRQ] RangingSlaveResponseDone: Ranging response complete (slave)");      }
    // Bit 8: RangingSlaveRequestDiscard
    if (irqStatus & (1 << 8)) {        ESP_LOGI(TAG, "[IRQ] RangingSlaveRequestDiscard: Ranging request discarded (slave)");    }
    // Bit 9: RangingMasterResultValid
    if (irqStatus & (1 << 9)) {        ESP_LOGI(TAG, "[IRQ] RangingMasterResultValid: Ranging result valid (master)");          }
    // Bit 10: RangingMasterTimeout
    if (irqStatus & (1 << 10)) {        ESP_LOGI(TAG, "[IRQ] RangingMasterTimeout: Ranging timeout (master)");                  }
    // Bit 11: RangingMasterRequestValid
    if (irqStatus & (1 << 11)) {        ESP_LOGI(TAG, "[IRQ] RangingMasterRequestValid: Ranging request valid (slave)");        }
    // Bit 12: CadDone
    if (irqStatus & (1 << 12)) {        ESP_LOGI(TAG, "[IRQ] CadDone: Channel activity detection complete");    }
    // Bit 13: CadDetected
    if (irqStatus & (1 << 13)) {        ESP_LOGI(TAG, "[IRQ] CadDetected: Channel activity detected");          }
    // Bit 14: RxTxTimeout
    if (irqStatus & (1 << 14)) {        ESP_LOGI(TAG, "[IRQ] RxTxTimeout: Rx or Tx timeout");          }
    // Bit 15: PreambleDetected
    if (irqStatus & (1 << 15)) {        ESP_LOGI(TAG, "[IRQ] PreambleDetected: Preamble detected");    }
}
//-------------------------------------------------------------------------------------------
int LoRaSX128X::packet_rssi(void) {
    uint8_t cmdGetPacketStatus[7] = {CMD_GET_PACKET_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t packetStatus[7] = {0};  

    spi_transaction_t t = {
        .length = sizeof(cmdGetPacketStatus) * 8,
        .rxlength = sizeof(packetStatus) * 8,
        .tx_buffer = cmdGetPacketStatus,        
        .rx_buffer = packetStatus
    };
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI packet_rssi transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG
    else{ ESP_LOGE(TAG, "SPI packet_rssi transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
    int rssi = packetStatus[2];
    return rssi - 137;
}
//----------------------------------------------------------------------------------------------------------------------------------------
float LoRaSX128X::packet_snr(void) {
    uint8_t cmdGetPacketStatus[7] = { CMD_GET_PACKET_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t packetStatus[7] = {0};

    spi_transaction_t t = {
        .length = sizeof(cmdGetPacketStatus) * 8,
        .rxlength = sizeof(packetStatus) * 8,
        .tx_buffer = cmdGetPacketStatus,        
        .rx_buffer = packetStatus
    };
    waitWhileBusy();
    gpio_set_level(_pinCS, 0);
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI packet_snr transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG
    else { ESP_LOGE(TAG, "SPI packet_snr transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();
    int8_t snr = (int8_t)packetStatus[3];
    return (float)snr / 4.0f;  // SX1280 SNR is in 0.25 dB steps
}
//----------------------------------------------------------------------------------------------------------------------------------------
/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int LoRaSX128X::readPacket(uint8_t *buf, int size) {

    // RxPacketStatus rps = getPacketStatus(); 
    RxBufferStatus rbs = getRxBufferStatus();
    // The second byte of the response contains the length of the received packet
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, "RxBufferStatus : rxlength = 0x%02X, start = 0x%02X.", rbs.rxLength, rbs.start);
#endif
    int len = readByteRegister(0x0901);
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, " Received %d bytes.", len);
#endif
    if (len > size) {
#ifdef LORA24_DEBUG
        ESP_LOGW(TAG, " Received (%d) is greater than buffer size (%d).", len, size);
#endif
        len = size;  // Truncate to fit buffer size
    }
#ifdef LORA24_DEBUG
    ESP_LOGW(TAG, " Reading %d bytes.", len);
#endif
    /*
     * Step 3: Read the received packet from the buffer.
     */
    read_buffer(rbs.start, buf, 255);
    
    return len;  // Return the length of received data
}

#define STATUS_PRINT_DELAY_MS 50 

int LoRaSX128X::receive_packet(uint8_t *buf, int size) {
    clear_irq();
    set_dio_mapping(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, 0x00, 0x00); // driver->lora->set_dio_mapping(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0x00, 0x00); //  IRQ_RX_DONE | IRQ_TX_DONE | 
    // printStatus();
    uint8_t buffer[256] = { 0x00 };
    buffer[255] = 0x00;
    setRx(5000);
    // printStatus();
    int16_t status = 0x00;
    uint32_t startTime = xTaskGetTickCount();  // Capture the start time
    uint32_t timeoutTicks = pdMS_TO_TICKS(5000);  // Timeout of 1000 ms

    while ((status & (IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT)) == 0) {
        status = getIrq();  // Check the IRQ status
        // printStatus();
        // Check if the RX timeout has been exceeded
        if ((xTaskGetTickCount() - startTime) > timeoutTicks) {
            ESP_LOGW(TAG, "------------------------------------------------------- ! RX operation timed out.");
            break;  // Break the loop if the timeout is reached
        }

        ESP_LOGW(TAG, "Waiting for packet-receive end...");
        // Delay for 5 seconds before checking the status again
        vTaskDelay(pdMS_TO_TICKS(STATUS_PRINT_DELAY_MS));
    }

    // Handle status after the loop (e.g., RX_DONE, RX_TIMEOUT, etc.)
    if (status & IRQ_RX_DONE) {
        ESP_LOGI(TAG, " ------------------------------------------------------------ Packet received successfully.");
        readPacket(buffer, 0xFF);
        
        memcpy(buf, buffer, size);
        
        return size;
    } else if (status & IRQ_RX_TX_TIMEOUT) {
        ESP_LOGW(TAG, " ----------------------------------------------------- RX timeout occurred.");
    } else {
        ESP_LOGW(TAG, " ----------------------------------------------------- Unknown status.");
    }
    return 0;
}
int LoRaSX128X::receiveACK(ACKPacket *ack, uint16_t timeOut) {
    clear_irq();
    set_dio_mapping(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, 0x00, 0x00); // driver->lora->set_dio_mapping(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0x00, 0x00); //  IRQ_RX_DONE | IRQ_TX_DONE | 
    // printStatus();
    uint8_t buffer[256] = { 0x00 };
    buffer[255] = 0x00;
    setRx(timeOut);
    // printStatus();
    int16_t status = 0x00;
    uint32_t startTime = xTaskGetTickCount();  // Capture the start time
    uint32_t timeoutTicks = pdMS_TO_TICKS(timeOut);  // Timeout of 1000 ms

    while ((status & (IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT)) == 0) {
        status = getIrq();  // Check the IRQ status
        // printStatus();
        // Check if the RX timeout has been exceeded
        if ((xTaskGetTickCount() - startTime) > timeoutTicks) {
            ESP_LOGW(TAG, "------------------------------------------------------- ! RX operation timed out.");
            break;  // Break the loop if the timeout is reached
        }

        ESP_LOGW(TAG, "Waiting for packet-receive end...");
        // Delay for 5 seconds before checking the status again
        vTaskDelay(pdMS_TO_TICKS(STATUS_PRINT_DELAY_MS));
    }

    // Handle status after the loop (e.g., RX_DONE, RX_TIMEOUT, etc.)
    if (status & IRQ_RX_DONE) {
        ESP_LOGI(TAG, " ------------------------------------------------------------ Packet received successfully.");
        readPacket(buffer, 0xFF);
        
        memcpy(ack, buffer, sizeof(ACKPacket));
        
        return sizeof(ACKPacket);
    } else if (status & IRQ_RX_TX_TIMEOUT) {
        ESP_LOGW(TAG, " ----------------------------------------------------- RX timeout occurred.");
    } else {
        ESP_LOGW(TAG, " ----------------------------------------------------- Unknown status.");
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------------
/**
 * Returns non-zero if there is data to read (packet received).
 */
int LoRaSX128X::received(void) {
    uint8_t cmdGetIrqStatus[] = {CMD_GET_IRQ_STATUS, 0x00, 0x00};
    uint8_t irqResponse[3] = {0};  // To store the response

    spi_transaction_t t = {
        .length = sizeof(cmdGetIrqStatus) * 8,
        .rxlength = sizeof(irqResponse) * 8,
        .tx_buffer = cmdGetIrqStatus,        
        .rx_buffer = irqResponse
    };
    // waitWhileBusy(_pinBusy);
    waitWhileBusy();  // Wait until the module is ready
    gpio_set_level(_pinCS, 0);
#if SPI_TRANSMIT
    esp_err_t ret = spi_device_transmit(__spi, &t);
#else
    esp_err_t ret = spi_device_polling_transmit(__spi, &t);
#endif
    gpio_set_level(_pinCS, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "----------------------------------- > SPI received transaction failed: %s", esp_err_to_name(ret)); }
#ifdef LORA24_DEBUG
    else { ESP_LOGW(TAG, "SPI received transaction succeeded: %s", esp_err_to_name(ret)); }
#endif
    waitWhileBusy();  // Wait until the module is ready
    uint16_t irqStatus = (irqResponse[1] << 8) | irqResponse[2];
    
    // Check if RX_DONE interrupt is set
    if (irqStatus & IRQ_RX_DONE) {
        return 1;  // Data is available
    }
    return 0;  // No data available
}
//----------------------------------------------------------------------------------------------------------------------------------------
