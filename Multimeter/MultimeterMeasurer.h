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
    rangeUpTimer_.start();
    rangeDownTimer_.start();
  }

  // Reads the voltage, returning true if the conversion was successful and false otherwise.
  // Voltage is 1V = kVoltageDenominator counts.
  bool readVoltageMv(int32_t* voltageOut = NULL, int32_t* rawAdcOut = NULL, uint16_t* rangeDivideOut = NULL) {
    int32_t adcValue;
    if (!adc_.readRaw24(&adcValue)) {
      return false;
    }

    uint16_t rangeDivide = measureRangeDivide_[getRange()];
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

  uint8_t getRange() {
    uint8_t rangeIndex = 0;
    for (size_t i=0; i<MeasureRangeBits; i++) {
      rangeIndex |= (measureRange_[i]->read() == 1) << i;
    }
    return rangeIndex;
  }

  void setRange(uint8_t rangeBits) {
    for (size_t i=0; i<MeasureRangeBits; i++) {
      measureRange_[i]->write((rangeBits & (1 << i)) != 0);
    }
  }

  void autoRange(int32_t adcValue) {
    uint32_t adcVolts = abs((int64_t)adcValue * kVoltageDenominator * kVref / kAdcCounts / kVrefDenominator);
    uint8_t currRange = getRange();
    uint32_t downRangeThreshold = UINT32_MAX;  // default that can't ever be triggered, if we're at lowest range
    uint32_t upRangeThreshold = kRangeMaxVoltage * kRangeUpThreshold / kRangeThresholdDenominator;

    if (currRange < (MeasureRangeCount - 1)) {  // if it's possible to shift down a range
      uint32_t currRangeFactor = (uint64_t)measureRangeDivide_[currRange] * kRangeThresholdDenominator / measureRangeDivide_[currRange + 1];
      downRangeThreshold = kRangeMaxVoltage * kRangeDownThreshold * kRangeThresholdDenominator / kRangeThresholdDenominator / currRangeFactor;
    }

    if (adcVolts > upRangeThreshold && currRange > 0) {
      if (rangeUpTimer_.elapsed_time().count() >= kRangeUpMs * 1000) {
        setRange(currRange - 1);
        rangeUpTimer_.reset();
      }
      rangeDownTimer_.reset();
    } else if (adcVolts < downRangeThreshold && currRange < (MeasureRangeCount - 1)) {
      if (rangeDownTimer_.elapsed_time().count() >= kRangeDownMs * 1000) {
        setRange(currRange + 1);
        rangeDownTimer_.reset();
      }
      rangeUpTimer_.reset();
    } else {
      rangeUpTimer_.reset();
      rangeDownTimer_.reset();
    }
  }

  static const uint32_t kVoltageDenominator = 1000;

protected:
  Mcp3561 adc_;
  uint16_t measureRangeDivide_[MeasureRangeCount];
  DigitalOut* measureRange_[MeasureRangeBits];

  static const uint32_t kVref = 2400;  // By default +/-2% at 25C
  static const uint32_t kVrefDenominator = 1000;
  static const int32_t kAdcCounts = 1 << 23;

  // Ranging control, note up/down is defined in terms of measurement range not control bits (which is inverted)
  Timer rangeUpTimer_, rangeDownTimer_;
  static const uint32_t kRangeMaxVoltage = 1650;  // max absolute ADC voltage, scaled by kVoltageDenominator
  static const uint32_t kRangeThresholdDenominator = 1000;
  static const uint32_t kRangeUpThreshold = 950;  // theshold of current max voltage before we up a range
  static const uint32_t kRangeDownThreshold = 900;  // threshold of previous max voltage before we down a range
  static const uint16_t kRangeUpMs = 0;  // delay beacuse we can move up a range, intentionally lower than RangeDown
  static const uint16_t kRangeDownMs = 100;  // delay before we can move down a range
};

#endif
