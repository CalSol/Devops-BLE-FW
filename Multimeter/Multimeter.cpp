#include <events/mbed_events.h>
#include <mbed.h>
#include <ble/BLE.h>
#include <ble/services/HealthThermometerService.h>
#include <ble/services/DeviceInformationService.h>

#include "USBSerial.h"

#include "ButtonGesture.h"
#include "RgbActivityLed.h"
#include "Mcp3201.h"
#include "Mcp3561.h"
#include "StatisticalCounter.h"
#include "LongTimer.h"

#include "MultimeterMeasurer.h"
#include "MultimeterDriver.h"

#include "StringService.h"
#include "NusService.h"
#include "MultimeterService.h"

#include "St7735sGraphics.h"
#include "DefaultFonts.h"
#include "Widget.h"

// for itoa
#include <stdio.h>
#include <stdlib.h>

// Example currently copy-pasted from https://github.com/platformio/platform-nordicnrf52/blob/master/examples/mbed-rtos-ble-thermometer/src/main.cpp
// Note Apache license.
// Will be removed once this is sufficiently refactored.

Timer UsTimer;
DigitalOut LedR(P1_11), LedG(P1_12), LedB(P1_13);
RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

PwmOut Speaker(P0_14);

DigitalOut GateControl(P0_25, 1);  // power gate

DigitalIn Switch0(P1_2);  // overlaid with power switch
ButtonGesture Switch0Gesture(Switch0);
DigitalIn Switch1(P0_27, PinMode::PullUp);  // up
ButtonGesture Switch1Gesture(Switch1);
DigitalIn Switch2(P1_10, PinMode::PullUp);  // down
ButtonGesture Switch2Gesture(Switch2);

SPI AdcSpi(P1_9, P0_8, P0_13);  // mosi, miso, sck
DigitalOut AdcCs(P0_15, 1);
Mcp3561 Adc(AdcSpi, AdcCs);
// Measure select options: TBD
DigitalOut MeasureRange0(P0_19);
DigitalOut MeasureRange1(P0_21);
DigitalOut InNegControl(P0_17, 1);  // 0 = GND, 1 = divider
// MultimeterMeasurer Meter(Adc, MeasureSelect, InNegControl);

DigitalOut DriverEnable(P0_24);  // 1 = enable driver
PwmOut DriverControl(P0_23);  // current driver setpoint
DigitalOut DriverRange0(P0_22);
DigitalOut DriverRange1(P0_20);
// MultimeterDriver Driver(DriverEnable, DriverControl);

BufferedSerial SwdUart(P1_0, NC, 115200);  // tx, rx
FileHandle *mbed::mbed_override_console(int) {  // redirect printf to SWD UART pins
    return &SwdUart;
}

USBSerial UsbSerial(false, 0x1209, 0x0001, 0x0001);


//
// LCD and widgets
//
SPI LcdSpi(P0_26, NC, P0_1);  // mosi, miso, sck
DigitalOut LcdCs(P0_0, 1);
DigitalOut LcdRs(P0_29);
DigitalOut LcdReset(P1_15, 0);
St7735sGraphics<160, 80, 1, 26> Lcd(LcdSpi, LcdCs, LcdRs, LcdReset);
TimerTicker LcdUpdateTicker(100 * 1000, UsTimer);

const uint8_t kContrastActive = 255;
const uint8_t kContrastStale = 191;

const uint8_t kContrastBackground = 191;

TextWidget widVersionData("BLE DMM", 0, Font5x7, kContrastActive);
TextWidget widBuildData("  " __DATE__, 0, Font5x7, kContrastBackground);
Widget* widVersionContents[] = {&widVersionData, &widBuildData};
HGridWidget<2> widVersionGrid(widVersionContents);

TextWidget widSerial(" ", 0, Font5x7, kContrastBackground);

TextWidget widEnable("     ", 0, Font5x7, kContrastStale);
LabelFrameWidget widEnableFrame(&widEnable, "ENABLE", Font3x5, kContrastBackground);

StaleNumericTextWidget widMeasV(0, 2, 100 * 1000, Font5x7, kContrastActive, kContrastStale, Font3x5, 1000, 2);
LabelFrameWidget widMeasVFrame(&widMeasV, "MEAS V", Font3x5, kContrastBackground);

StaleNumericTextWidget widMeasI(0, 2, 100 * 1000, Font5x7, kContrastActive, kContrastStale, Font3x5, 1000, 2);
LabelFrameWidget widMeasIFrame(&widMeasI, "MEAS I", Font3x5, kContrastBackground);

Widget* widMeasContents[] = {&widEnableFrame, &widMeasVFrame, &widMeasIFrame};
HGridWidget<3> widMeas(widMeasContents);

Widget* widMainContents[] = {&widVersionGrid, &widSerial, &widMeas};
VGridWidget<3> widMain(widMainContents);


// BLE comms
const static char DEVICE_NAME[] = "DuckyMultimeter";
const size_t kEventQueueSize = 16;
static events::EventQueue event_queue(kEventQueueSize * EVENTS_EVENT_SIZE);

class ThermometerDemo : ble::Gap::EventHandler {
public:
    ThermometerDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _adv_data_builder(_adv_buffer) { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &ThermometerDemo::on_init_complete);
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            // print_error(params->error, "Ble initialization failed.");
            return;
        }

        ble::own_address_type_t addr_type;
        ble::address_t address;
        _ble.gap().getAddress(addr_type, address);
        printf("Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
            address.data()[5], address.data()[4], address.data()[3], address.data()[2], address.data()[1], address.data()[0]);

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        const UUID deviceInformationService = GattService::UUID_DEVICE_INFORMATION_SERVICE;
        const UUID uartService = NusService::kServiceUuid;
        const UUID services[] = {deviceInformationService, uartService};

        _adv_data_builder.setFlags();
        // _adv_data_builder.setLocalServiceList(mbed::make_Span(services, 2));  // somehow breaks names
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::DIGITAL_PEN);
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            // print_error(error, "_ble.gap().setAdvertisingParameters() failed");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            // print_error(error, "_ble.gap().setAdvertisingPayload() failed");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            // print_error(error, "_ble.gap().startAdvertising() failed");
            return;
        }
    }

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
         _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }

private:
    BLE &_ble;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}


// Application-specific temporary
StatisticalCounter<uint32_t, uint64_t> AdcStats;
StatisticalCounter<int32_t, int64_t> VoltageStats;
Timer ConvTimer;


int main() {
  // Set SWO pin into GPIO mode
  NRF_CLOCK->TRACECONFIG = 0;

  Speaker.period_us(10);
  Speaker.write(0.5);

  printf("\r\n\r\n\r\n");
  printf("BLE Multimeter\n");
  printf("Built " __DATE__ " " __TIME__ "\n");

  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(schedule_ble_events);

  ThermometerDemo demo(ble, event_queue);
  demo.start();

  VoltmeterService bleVoltmeter(ble, 0x183B);
  DeviceInformationService bleDeviceInfo(ble, "Ducky", "Multimeter", "0001",
                                      "rv1", __DATE__ " " __TIME__, "NA");
  NusService bleConsole(ble);  

  UsTimer.start();

  Timer timer;
  timer.start();

  Timer audioTimer;
  audioTimer.start();

  Timer audioTimer2;
  audioTimer2.start();

  ConvTimer.start();

  LedR = 1;
  LedG = 1;
  LedB = 1;

  LcdSpi.format(8, 0);
  Lcd.init();

  Adc.init();
  Adc.startConversion();

  printf("ADC Configs 0=%02x, 1=%02x, 2=%02x, 3=%02x\n", 
    Adc.readReg8(Mcp3561::kRegister::CONFIG0), Adc.readReg8(Mcp3561::kRegister::CONFIG1),
    Adc.readReg8(Mcp3561::kRegister::CONFIG2), Adc.readReg8(Mcp3561::kRegister::CONFIG3));

  // Driver.enable();
  // Driver.setCurrent(2000);

  while (1) {
    if (audioTimer2.elapsed_time().count() >= 10) {
      float phase = audioTimer.elapsed_time().count() / 1000000.0 * 440.0 * 2 * 3.14159;
      Speaker.write(0.5 + 0.5 * sin(phase));
      audioTimer2.reset();
    }

    // if (audioTimer2.elapsed_time().count() >= 1000) {
    //     if (audioTimer2.elapsed_time().count() % 9090 > 4545) {
    //         Speaker.write(0.25);
    //     } else {
    //         Speaker.write(0.75);
    //     }
    // }

    if (UsbSerial.connected()) {
      StatusLed.setIdle(RgbActivity::kGreen);
    } else if (UsbSerial.configured()) {
      StatusLed.setIdle(RgbActivity::kYellow);
    } else {
      UsbSerial.connect();
      StatusLed.setIdle(RgbActivity::kOff);
    }

    switch (Switch0Gesture.update()) {
      case ButtonGesture::Gesture::kClickRelease:  // test code
        // ThermService.updateTemperature(LedR == 1 ? 30 : 25);
        break;
      case ButtonGesture::Gesture::kHoldTransition:  // long press to shut off
        GateControl = 0;
        break;
      default: break;
    }

    switch (Switch1Gesture.update()) {
      default: break;
    }

    switch (Switch2Gesture.update()) {
      case ButtonGesture::Gesture::kClickRelease:
        // MeasureSelect = !MeasureSelect;
        break;
      case ButtonGesture::Gesture::kHoldTransition:  // long press to switch into driver mode
        InNegControl = !InNegControl;
        if (InNegControl == 0) {
          // Driver.setCurrent(1000);
          // MeasureSelect = 1;  // set to direct measurement
          // Driver.enable(true);
        } else {
          // Driver.enable(false);
        }
        break;
      default: break;
    }
      
    uint32_t adcValue;
    int32_t voltage;

    if (Adc.readRaw24(&adcValue)) {
      StatusLed.pulse(RgbActivity::kGreen);
      printf("ADC <- %lu\n", adcValue);
      Adc.startConversion();
    }

    // if (Meter.readVoltageMv(&voltage, &adcValue)) {
    //   AdcStats.addSample(adcValue);
    //   VoltageStats.addSample(voltage);
    //   // printf("% 3lims    ADC=%li lsb    V=%li mV\n", 
    //   //   ConvTimer.read_ms(), adcValue, voltage);
    //   // ConvTimer.reset();
    //   if (DriverEnable == 1) {
    //     StatusLed.pulse(RgbActivity::kRed);
    //   } else {
    //     // if (MeasureSelect == 1) {  // direct
    //       StatusLed.pulse(RgbActivity::kCyan);
    //     // } else {  // divided
    //     //   StatusLed.pulse(RgbActivity::kGreen);
    //     // }
    //   } 
    // }

    if (timer.read_ms() >= 1000) {
      timer.reset();

      auto adcStats = AdcStats.read();
      auto voltageStats = VoltageStats.read();
      AdcStats.reset();
      VoltageStats.reset();

      bleVoltmeter.writeVoltage(voltageStats.avg);

      // debugging stuff below
      // printf("MS=%i, NC=%i, ADC(%u) = %lu - %lu - %lu (%lu)    V(%u) = %li - %li - %li (%li)\n", 
      //     MeasureSelect.read(), InNegControl.read(),
      //     adcStats.numSamples, adcStats.min, adcStats.avg, adcStats.max, 
      //     adcStats.max-adcStats.min,
      //     voltageStats.numSamples, voltageStats.min, voltageStats.avg, voltageStats.max, 
      //     voltageStats.max-voltageStats.min);

      if (UsbSerial.connected()) {
        UsbSerial.printf("\n");
      }

      char voltsStr[128];
      itoa(voltageStats.avg, voltsStr, 10);
      strcat(voltsStr, " mV\n");
      bleConsole.write(voltsStr);
    }

    event_queue.dispatch_once();

    if (LcdUpdateTicker.checkExpired()) {
      Lcd.clear();
      widMain.layout();
      widMain.draw(Lcd, 0, 0);
      Lcd.update();
    }

    StatusLed.update();
  }
}
