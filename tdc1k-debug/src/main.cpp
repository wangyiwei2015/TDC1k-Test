#include "TDC1000.h"
#include <driver/ledc.h>

//#define PIN_TDC1000_START     (2)
//#define PIN_TDC1000_STOP      (3)
#define PIN_TDC1000_TRIGGER   (4)
#define PIN_TDC1000_RESET     (5)
//#define PIN_TDC1000_CHSEL     (7)
//#define PIN_TDC1000_ERRB      (8)
#define PIN_TDC1000_SPI_CS    (9)
#define PIN_TDC1000_CLKIN    (17)
#define TDC1000_CLKIN_FREQ_HZ  8000000
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div2)

//#include <TimerOne.h>
//#define PWM_CYCLE_US (1000000/TDC1000_CLKIN_FREQ_HZ)

static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

void setup() {
    Serial.begin(115200);
    while(!Serial.available());
    delay(1000); Serial.flush();
    Serial.println(F("-- Starting TDC1000 test --"));
    while (not usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        delay(2000);
    }
    Serial.println(F("TDC1000 init OK"));

    //usafe.dumpSettings();
    delay(1000);

    bool ok = true;
    ok &= usafe.setTriggerEdge(true);
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 6 /*pulses*/, 31 /*shift*/, true /*damping*/);
    ok &= usafe.setRx(false /*multiEcho*/);
    ok &= usafe.setRxSensitivity(TDC1000::RxDacEchoThreshold::m220mV, TDC1000::RxPgaGain::g21dB, TDC1000:: RxLnaFbMode::capacitive);
    ok &= usafe.setRepeat(TDC1000::TxRxCycles::x1, 0 /*expected pulses*/);
    ok &= usafe.setTofMeasuementShort(
        TDC1000::T0::ClkInDiv1, TDC1000::TxAutoZeroPeriod::T0x64,
        TDC1000::TxBlankPeriod::T0x16, TDC1000::TxEchoTimeoutPeriod::disabled);
    ok &= usafe.setMeasureTOF(TDC1000::TxRxChannel::Channel1, TDC1000::TofMode::Mode0); // Mode2
    //ok &= usafe.setMeasureTOF(TDC1000::TxRxChannel::Swap, TDC1000::TofMode::Mode2);
    usafe.dumpSettings(TDC1000_CLKIN_FREQ_HZ);

    if (not ok) {
        Serial.println(F("Failed to configure TDC1000"));
        while(1);
    }

    // 配置PWM
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT, // 1位分辨率
        .timer_num = LEDC_TIMER_0,
        .freq_hz = TDC1000_CLKIN_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .gpio_num = PIN_TDC1000_CLKIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,  // 占空比50%
        .hpoint = 0
    };
    ledc_channel_config(&ch_cfg);

    //pinMode(PIN_TDC1000_START, INPUT);
    //pinMode(PIN_TDC1000_STOP, INPUT);

    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);

    Serial.flush();
}

void loop() {
    usafe.clearErrorFlags();
    usafe.resetStatemachine();

    while(!Serial.available());
    String input = Serial.readString();
    
    if(input == "t") {
        Serial.println("output signal");
        // Trigger new measurement
        digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
        digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    }
}
