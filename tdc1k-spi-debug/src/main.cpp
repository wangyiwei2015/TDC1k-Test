#include <Arduino.h>
#include <HardwareSerial.h>
#include "TDC1000.h"

#define TRIG A4
#define TDCEN A5
#define TXCLK 9

#define PIN_TDC1000_RESET      (2)
#define PIN_TDC1000_SPI_CS     (3)
#define TDC1000_CLKIN_FREQ_HZ  8000000
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div2)
static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

void setup() {
    pinMode(TDCEN, OUTPUT); digitalWrite(TDCEN, 0);
    pinMode(TRIG, OUTPUT); digitalWrite(TRIG, 0);
    pinMode(TXCLK, OUTPUT);

    Serial.begin(115200);
    while(!Serial) delay(200);
    delay(500); Serial.flush();
    Serial.println(F("-- Starting TDC1000 test --"));
    while (not usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        delay(2000);
    }
    Serial.println(F("TDC1000 init OK"));

    digitalWrite(TDCEN, 0);
    tone(TXCLK, 800000);

    delay(1000);

    bool ok = true;
    ok &= usafe.setTriggerEdge(true);
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 6 /*pulses*/, 0 /*shift*/, true /*damping*/);
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

    Serial.flush();
}

void loop() {
    usafe.clearErrorFlags();
    usafe.resetStatemachine();
    delay(1000);
    Serial.println("burst");
    digitalWrite(TRIG, 1);
    delay(1);
    digitalWrite(TRIG, 0);
    delay(10);
    bool wk, no, hi;
    usafe.getErrorFlags(wk, no, hi);
    if(wk) Serial.println("weak");
    if(no) Serial.println("no signal");
    if(hi) Serial.println("too hi");
}
