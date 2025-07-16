#include "TDC1000.h"

#define PIN_TDC1000_ERRB      (2)
#define PIN_TDC1000_START     (3)
#define PIN_TDC1000_STOP      (A5)
#define PIN_TDC1000_RESET     (4)
#define PIN_TDC1000_TRIGGER   (5)
#define PIN_TDC1000_EN        (6)
#define PIN_TDC1000_CHSEL     (7)
#define PIN_TDC1000_SPI_CS    (A4)
#define PIN_TDC1000_CLKIN     (11)
#define TDC1000_CLKIN_FREQ_HZ  8000000
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div2)

static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

void onStart() {
    Serial.println("start sigbnal");
}

void onError() {
    Serial.println("error signal");
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(200);
    delay(500); Serial.flush();
    Serial.println(F("-- Starting TDC1000 test --"));
    while (not usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        delay(2000);
    }
    Serial.println(F("TDC1000 init OK"));

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

    pinMode(PIN_TDC1000_START, INPUT_PULLUP);
    pinMode(PIN_TDC1000_STOP, INPUT_PULLUP);
    pinMode(PIN_TDC1000_CHSEL, OUTPUT);
    pinMode(PIN_TDC1000_ERRB, INPUT_PULLUP);
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);
    pinMode(PIN_TDC1000_CLKIN, OUTPUT);
    pinMode(13, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_TDC1000_START), onStart, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_TDC1000_ERRB), onError, RISING);
    analogWrite(PIN_TDC1000_CLKIN, 128);

    Serial.flush();
}

inline void burst(int time = 1) {
    Serial.println("== BURST ==");
    digitalWrite(13, 1);
    digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
    delay(time);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    digitalWrite(13, 0);
}

void loop() {
    usafe.clearErrorFlags();
    usafe.resetStatemachine();

    delay(1000);
    burst();
    return;

    while(!Serial.available());
    String input = Serial.readString();
    
    if(input == "t") {
        Serial.println("output signal");
        burst();
    }
    //usafe.getErrorFlags
}
