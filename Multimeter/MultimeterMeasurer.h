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
  MultimeterMeasurer(Mcp3561& adc, uint16_t measureRangeDivide[], DigitalOut* measureRange[]) :
      adc_(adc) {
    for (size_t i=0; i<MeasureRangeCount; i++) {
      measureRangeDivide_[i] = measureRangeDivide[i];
    }
    for (size_t i=0; i<MeasureRangeBits; i++) {
      measureRange_[i] = measureRange[i];
    }
    rangeTimer_.start();
  }

  // Reads the voltage, returning true if the conversion was successful and false otherwise.
  // Voltage is 1V = kVoltageDenominator counts.
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

    int32_t voltage = (int64_t)adcValue * kVoltageDenominator * rangeDivide * kVref / kAdcCounts / kVrefDenominator;

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

  void autoRange(int32_t rawAdc) {

  }

  static const uint32_t kVoltageDenominator = 1000;

protected:
  Mcp3561 adc_;
  uint16_t measureRangeDivide_[MeasureRangeCount];
  DigitalOut* measureRange_[MeasureRangeBits];

  static const uint32_t kVref = 2400;  // By default +/-2% at 25C
  static const uint32_t kVrefDenominator = 1000;
  static const int32_t kAdcCounts = 1 << 23;

  // Ranging control
  Timer rangeTimer_;
  static const uint32_t kRangeThresholdDenominator = 1000;
  static const uint32_t kRangeUpThreshold = 950;  // theshold of current max voltage before we up a range
  static const uint32_t kRangeDownThreshold = 850;  // threshold of previous max voltage before we down a range
  static const uint16_t kRangeUpMs = 0;  // delay beacuse we can move up a range, intentionally lower than RangeDown
  static const uint16_t kRangeDownMs = 100;  // delay before we can move down a range
};

#endif
