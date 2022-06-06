#ifndef __MULTIMETER_MEASURER_H__
#define __MULTIMETER_MEASURER_H__
#include <mbed.h>
#include "Mcp3561.h"


/**
 * Volts measurement stage for the multimeter.
 * Optional auto-ranging.
 */
template <size_t MeasureRangeCount, uint8_t MeasureRangeBits>
class MultimeterMeasurer {
public:
  MultimeterMeasurer(Mcp3561& adc, uint16_t measureRangeDivide[], DigitalOut* measureRange[], DigitalOut& referenceSelect) :
      adc_(adc), referenceSelect_(referenceSelect) {
    for (size_t i=0; i<MeasureRangeCount; i++) {
      measureRangeDivide_[i] = measureRangeDivide[i];
    }
    for (size_t i=0; i<MeasureRangeBits; i++) {
      measureRange_[i] = measureRange[i];
    }
  }

  // Reads the voltage in milli-volts, returning true if the conversion was successful and false otherwise
  bool readVoltageMv(int32_t* voltageOut = NULL, int32_t* rawAdcOut = NULL, uint16_t* rangeDivideOut = NULL) {
    int32_t adcValue;
    if (!adc_.readRaw24(&adcValue)) {
      return false;
    }

    uint8_t rangeIndex = 0;
    for (size_t i=0; i<MeasureRangeBits; i++) {
      rangeIndex |= (measureRange_[i]->read() == 1) << i;
    }
    uint16_t rangeDivide = measureRangeDivide_[rangeIndex];

    int32_t voltage = (int64_t)adcValue * 1000 * rangeDivide * kVref / kAdcCounts / kVrefDenominator;

    if (voltageOut != NULL) {
      *voltageOut = voltage;
    }
    if (rawAdcOut != NULL) {
      *rawAdcOut = adcValue;
    }
    if (rangeDivideOut != NULL) {
      *rangeDivideOut = rangeDivide; 
    }

    return true;
  }

protected:
  Mcp3561 adc_;
  uint16_t measureRangeDivide_[MeasureRangeCount];
  DigitalOut* measureRange_[MeasureRangeBits];
  // DigitalOut measureSelect_;  // 0 = 1M/10k divider, 1: direct input
  DigitalOut referenceSelect_;  // 0 = GND, 1 = 1/2 divider (allows measuring negative voltages, 'virtual ground')

  static const uint32_t kVref = 2400;  // By default +/-2% at 25C
  static const uint32_t kVrefDenominator = 1000;
  static const int32_t kAdcCounts = 1 << 23;

  // static const int32_t kCalibrationDenominator = 1000;
  // int32_t adcDivIntercept_ = 0;
  // int32_t adcSlope1_ = (float)kCalibrationDenominator * kAdcCounts / kVref;
  // int32_t adcSlope100_ = (float)kCalibrationDenominator * kAdcCounts * (10.0/1010.0) / kVref;
};

#endif
