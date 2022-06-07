#include <mbed.h>

#ifndef __TICKER_SPEAKER_H__
#define __TICKER_SPEAKER_H__

class TickerSpeaker {
public:
  TickerSpeaker(PwmOut& pwm, uint32_t pwmPeriodUs) : pwm_(pwm) {
    pwm_.period_us(pwmPeriodUs);
  }

  // Play a square wave tone, of the specified frequency and amplitude, for the specified duration.
  // Zero duration means continuously play.
  // Zero frequency means to stop the previous time.
  void tone(uint32_t frequencyHz, float amplitude, uint32_t durationUs = 0) {
    if (frequencyHz == 0) {
      ticker_.detach();
      return;
    }
    amplitude_ = amplitude;
    uint32_t periodUs = 1000 * 1000 / frequencyHz / 2;
    if (durationUs == 0) {
      countsRemaining_ = 0;
    } else {
      countsRemaining_ = durationUs / periodUs;
      if (countsRemaining_ == 0) {
        countsRemaining_ = 1;
      }
    }
    ticker_.attach(callback(this, &TickerSpeaker::irq), std::chrono::microseconds(periodUs));
  }

protected:
  void irq() {
    if (countsRemaining_ > 0) {
      countsRemaining_ -= 1;
      if (countsRemaining_ == 0) {
        pwm_ = 0;  // must idle at zero - otherwise PWM signal may bleed into speaker
        ticker_.detach();
        return;
      }
    }

    if (!lastSpeaker) {
      pwm_ = amplitude_;
    } else {
      pwm_ = 0;
    }
    lastSpeaker = !lastSpeaker;
  }

  PwmOut& pwm_;
  Ticker ticker_;

  volatile uint32_t countsRemaining_ = 0;  // duration of tone in periods, 0 for infinitely looping
  volatile bool lastSpeaker = false;
  volatile float amplitude_ = 0;
};

#endif
