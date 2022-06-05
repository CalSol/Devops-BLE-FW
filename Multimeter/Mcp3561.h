#include <mbed.h>

#ifndef __MCP3561_H__
#define __MCP3561_H__


// MCP3561/2/4 24-bit SPI ADC
class Mcp3561 {
public:
  // per the packaging information on the datasheet, all devices have address b01,
  // but alternative addresses can be special orderred
  Mcp3561(SPI& spi, DigitalOut& cs, int address = 1, int frequency = 1000000) : 
      spi_(spi), cs_(cs), address_(address), frequency_(frequency) {
  }

  // reads ADC as a 24-bit value, returning whether the ADC had new data
  // if the ADC did not have new data, a new conversion will be started
  bool read_raw_u24(uint32_t* outValue) {

    return true;
  }

  uint8_t readReg8(uint8_t regAddr) {
    cs_ = 0;
    wait_ns(25);
    uint8_t status = spi_.write((address_ & 0x3) << 6 |
                                (regAddr & 0xf) << 2 | 
                                kCommandType::kStaticRead);

    wait_ns(50);
    cs_ = 1;

    return status;
  }


  enum kRegister {
    ADCMODE = 0x0,
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

protected:
  SPI& spi_;
  DigitalOut& cs_;
  int frequency_;
  uint8_t address_;  // 2 bit device address
};

#endif  // __MCP3561_H__
