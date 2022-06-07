#include <mbed.h>

#ifndef __MCP3561_H__
#define __MCP3561_H__


// MCP3561/2/4 24-bit SPI ADC
class Mcp3561 {
public:
  // per the packaging information on the datasheet, all devices have address b01,
  // but alternative addresses can be special orderred
  Mcp3561(SPI& spi, DigitalOut& cs, int address = 1) : 
      spi_(spi), cs_(cs), address_(address) {
  }

  // default init routine
  void init(uint8_t oversample = kOsr::k256) {
    writeReg8(kRegister::CONFIG0, 0xE2);  // internal VREF, internal clock w/ no CLK out, ADC standby
    writeReg8(kRegister::CONFIG1, (oversample & 0xf) << 2);
    writeReg8(kRegister::CONFIG3, 0x80);  // one-shot conversion into standby, 24b encoding
    writeReg8(kRegister::IRQ, 0x07);  // enable fast command and start-conversion IRQ, IRQ logic high
    // note IRQ logic high needed if IRQ is not externally pulled high

    writeReg8(kRegister::MUX, 0x01);  // IN+ = CH0, IN- = CH1
  }

  // (re)starts a conversion, which can be ready out with readRaw24
  uint8_t startConversion() {
    return fastCommand(kFastCommand::kStartConversion);
  }

  // resets the device using the reset fast command
  void fullReset() {
    writeReg8(kRegister::IRQ, 0x03);  // enable fast command and start-conversion IRQ
    fastCommand(kFastCommand::kDeviceFullReset);
  }

  // sends a fast command, returning the status code
  uint8_t fastCommand(uint8_t fastCommandCode) {
    cs_ = 0;
    wait_ns(25);
    uint8_t status = spi_.write(((address_ & 0x3) << 6) |
                                ((fastCommandCode & 0xf) << 2) | 
                                kCommandType::kFastCommand);
    wait_ns(50);
    cs_ = 1;

    return status;
  }

  // tries to reads the ADC as a signed 24-bit value, returning whether the ADC had new data
  // does NOT start a new conversion
  bool readRaw24(int32_t* outValue) {
    bool valid = false;
    cs_ = 0;
    wait_ns(25);

    uint8_t status = spi_.write(((address_ & 0x3) << 6) |
                                ((kRegister::ADCDATA & 0xf) << 2) | 
                                kCommandType::kStaticRead);
    if ((status & 0x4) == 0) {  // STAT[2] /DataReady
      uint8_t dummy = 0;
      uint8_t data[3];
      spi_.write((char*)&dummy, 1, (char*)data, 3);
      *outValue = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
      if (*outValue & (1 << 23)) {  // sign extend
        *outValue |= (int32_t)0xff << 24;
      }
      valid = true;
    }

    wait_ns(50);
    cs_ = 1;

    return valid;
  }

  // writes 8 bits into a single register, returning the status code
  uint8_t writeReg8(uint8_t regAddr, uint8_t data) {
    uint8_t spiData[2];
    spiData[0] = ((address_ & 0x3) << 6) | ((regAddr & 0xf) << 2) | kCommandType::kIncrementalWrite;
    spiData[1] = data;
    uint8_t status;
    
    cs_ = 0;
    wait_ns(25);
    spi_.write((char*)spiData, 2, (char*)&status, 1);
    wait_ns(50);
    cs_ = 1;

    return status;
  }

  uint8_t readReg8(uint8_t regAddr) {
    uint8_t spiData[2];
    spiData[0] = ((address_ & 0x3) << 6) | ((regAddr & 0xf) << 2) | kCommandType::kStaticRead;
    spiData[1] = 0;
    uint8_t returnData[2];

    cs_ = 0;
    wait_ns(25);
    spi_.write((char*)spiData, 2, (char*)returnData, 2);
    wait_ns(50);
    cs_ = 1;

    return returnData[1];
  }

  enum kRegister {
    ADCDATA = 0x0,
    CONFIG0 = 0x1,
    CONFIG1 = 0x2,
    CONFIG2 = 0x3,
    CONFIG3 = 0x4,
    IRQ = 0x5,
    MUX = 0x6,
    SCAN = 0x7,
    TIMER = 0x8,
    OFFSETCAL = 0x9,
    GAINCAL = 0xA,
    LOCK = 0xD,
    CRCCFG = 0xF
  };

  enum kCommandType {
    kFastCommand = 0,
    kStaticRead = 1,
    kIncrementalWrite = 2,
    kIncrementalRead = 3
  };

  enum kFastCommand {
    kStartConversion = 0xA,  // ADC_MODE[1:0] = 0b11
    kStandbyMode = 0xB,  // ADC_MODE[1:0] = 0b10
    kShutdownMode = 0xC,  // ADC_MODE[1:0] = 0b00
    kFullShutdownMode = 0xD,  // CONFIG0[7:0] = 0x00
    kDeviceFullReset = 0xE  // resets entire register map
  };

  enum kOsr {
    k98304 = 0xf,
    k81920 = 0xe,
    k49152 = 0xd,
    k40960 = 0xc,
    k24576 = 0xb,
    k20480 = 0xa,
    k16384 = 0x9,
    k8192 = 0x8,
    k4096 = 0x7,
    k2048 = 0x6,
    k1024 = 0x5,
    k512 = 0x4,
    k256 = 0x3,  // default
    k128 = 0x2,
    k64 = 0x1,
    k32 = 0x0
  };

protected:
  SPI& spi_;
  DigitalOut& cs_;
  uint8_t address_;  // 2 bit device address
};

#endif  // __MCP3561_H__
