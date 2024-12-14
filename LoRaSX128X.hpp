#ifndef LORA_SX128X_H
#define LORA_SX128X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
}
#endif

#define TAG "LoRaSX128X"

// LoRa Commands
#define CMD_SET_STANDBY                 0x80
#define CMD_SET_SLEEP                   0x84
#define CMD_SET_RF_FREQUENCY            0x86
#define CMD_SET_PACKET_TYPE             0x8A
#define CMD_SET_MODULATION_PARAMS       0x8B
#define CMD_SET_PACKET_PARAMS           0x8C
#define CMD_SET_DIO_IRQ_PARAMS          0x8D
#define CMD_SET_TX_PARAMS               0x8E
#define CMD_CLEAR_IRQ_STATUS            0x97
#define CMD_GET_IRQ_STATUS              0x15
#define CMD_GET_PACKET_STATUS           0x1D
#define CMD_WRITE_REGISTER              0x18
#define CMD_READ_REGISTER               0x19
#define CMD_WRITE_BUFFER                0x1A
#define CMD_READ_BUFFER                 0x1B

// IRQ Masks
#define IRQ_TX_DONE                     0x0001
#define IRQ_RX_DONE                     0x0002
#define IRQ_RX_TX_TIMEOUT               0x4000
#define IRQ_ALL                         0xFFFF

// LoRa Modulation Parameters
#define LORA_SF5                        0x50
#define LORA_SF7                        0x70
#define LORA_BW_1600                    0x0A
#define LORA_CR_4_5                     0x01
#define LORA_CR_4_6                     0x02
#define LORA_IQ_NORMAL                  0x40

// LoRa Header Types
#define HEADER_TYPE_IMPLICIT            0x80
#define HEADER_TYPE_EXPLICIT            0x00

// LoRa CRC Modes
#define LORA_CRC_OFF                    0x00
#define LORA_CRC_ON                     0x20

// Register Definitions
#define REG_LORA_SYNC_WORD_MSB          0x0944
#define REG_LORA_SYNC_WORD_LSB          0x0945
#define REG_FIFO                        0x00
#define REG_PACKET_PARAMS               0x903

// Helper Structures
typedef struct {
    uint8_t rxLength;
    uint8_t start;
} RxBufferStatus;

typedef struct {
    uint8_t rssiSync;
    uint8_t snrPkt;
} RxPacketStatus;

typedef struct {
    uint8_t id;
    uint8_t value;
} ACKPacket;

struct LoRaParameters {
    uint32_t frequencyHz;          // Operating frequency in Hz
    uint8_t spreadingFactor;       // LoRa spreading factor (e.g., LORA_SF7)
    uint8_t bandwidth;             // LoRa bandwidth (e.g., LORA_BW_1600)
    uint8_t codingRate;            // LoRa coding rate (e.g., LORA_CR_4_6)
    uint8_t preambleLength;        // Preamble length
    uint8_t headerType;            // Header type (explicit/implicit)
    uint8_t payloadLength;         // Payload length
    uint8_t crcMode;               // CRC mode (enabled/disabled)
    uint8_t iqMode;                // IQ mode (normal/inverted)
    uint8_t syncWord;              // Sync word
};

#ifdef __cplusplus

class LoRaSX128X {
public:
    // Constructor
    LoRaSX128X(gpio_num_t pinMiso, gpio_num_t pinMosi, gpio_num_t pinSclk, gpio_num_t pinCS, 
               gpio_num_t pinRST, gpio_num_t pinBusy, int8_t txPower);

    // Initialization and Configuration
    int full_init(const LoRaParameters& params);
    void checkPinConfiguration();
    int waitWhileBusy();
    void setStandBy();
    void setRx(uint16_t timeOut);
    void setTx();
    void setSaveContext();
    void setLoRaMode();
    void setFrequencyInHz(uint32_t rfFrequency);
    void setModulationParameters(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate);
    void setPacketParameters(uint8_t preambleLength, uint8_t headerType, uint8_t payloadLength, 
                              uint8_t crcMode, uint8_t invertIQ);
    void setBufferBaseAddr();
    void setTxParameters(int8_t power, uint8_t rampTime);
    void setRegulatorMode(uint8_t mode);
    uint8_t setSyncWord(uint8_t syncWord);
    uint8_t readSyncWord();
    void reset();

    // SPI and Command Handling
    void sendCommandCode(const uint8_t* cmd, size_t cmdLength);
    void writeRegister(uint16_t reg, uint8_t* data, size_t len);
    void writeByteToRegister(uint16_t reg, uint8_t data);
    void writeWordToRegister(uint16_t reg, uint16_t data);
    uint8_t readByteRegister(uint16_t reg);
    uint16_t readWordRegister(uint16_t reg);
    void write_buffer(uint8_t startAddr, uint8_t* val, size_t len);
    void read_buffer(uint8_t offset, uint8_t* buffer, size_t size);

    // IRQ Handling
    void clear_irq();
    uint16_t getIrq();
    void printIRQStatus(uint16_t irqStatus);
    void set_dio_mapping(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);

    // Packet and Reception Handling
    int receive_packet(uint8_t* buf, int size);
    int receiveACK(ACKPacket* ack, uint16_t timeOut);
    int readPacket(uint8_t* buf, int size);
    int received();

    // Status and Debug
    uint8_t getStatus();
    uint8_t printStatus();
    void interpretStatus(uint8_t status);
    void readChipRegisters();

    // RSSI and SNR
    int packet_rssi();
    float packet_snr();

    // Fields
    uint8_t currentBandwidth;
    uint8_t currentCodingRate;
    uint8_t currentSpreadingFactor;
    uint8_t currentPreambleLength;
    uint8_t currentHeaderType;
    uint8_t currentCrcType;
    uint8_t currentPayloadLength;
    uint8_t currentIQInvertStatus;
    uint8_t currentSyncWord;

    gpio_num_t _pinMiso;
    gpio_num_t _pinMosi;
    gpio_num_t _pinSclk;
    gpio_num_t _pinCS;
    gpio_num_t _pinRST;
    gpio_num_t _pinBusy;
    int _txPower;

private:
    spi_device_handle_t _spiHandle;
    int _implicit;
};

#endif // __cplusplus

#endif // LORA_SX128X_H
