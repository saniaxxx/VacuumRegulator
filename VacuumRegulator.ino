#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <LiquidCrystal.h>

#define KEY_NONE 0
#define KEY_SELECT 1
#define KEY_UP 2
#define KEY_DOWN 3
#define KEY_LEFT 4
#define KEY_RIGHT 5
#define RELAY_PIN A3
#define SENSOR_PIN A5
#define RELAY_ACTIVE_LEVEL HIGH
#define DSP_CLR "   "
//#define HDCTL_REPORT_INTERVAL 1000

unsigned long timing;

struct {
    float setpoint;
    float hysteresis;
}
eeprom;

union {
    uint16_t raw[8];
    struct {
        uint16_t RSV, C1, C2, C3, C4, C5, C6, CRC;
    };
}
ms5611_prom;

LiquidCrystal dsp(8, 9, 4, 5, 6, 7);
unsigned long eeprom_save_ts;
byte dsp_state = 0;
byte relay_state = 0;
float pressure = 0;

void setup() {
    #ifdef HDCTL_REPORT_INTERVAL
    Serial.begin(9600);
    #endif
    wdt_enable(WDTO_60MS);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, !RELAY_ACTIVE_LEVEL);
    Wire.begin();
    dsp.begin(16, 2);
    dsp.clear();
    eeprom_init();
    dsp.print("PV: ");
    key_press(KEY_SELECT);
}

void loop() {
    unsigned long ts = millis();
  
    wdt_reset();
    key_poll();
    read_pressure();

    dsp.setCursor(4, 0);
    dsp.print(pressure);
    dsp.print(DSP_CLR);

    if (millis() - timing > 1000){
      timing = millis();
      float margin = eeprom.hysteresis / 2;
      if (pressure >= eeprom.setpoint + margin && !relay_state) {
          relay_state = 1;
          digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL);
      } else if (pressure <= eeprom.setpoint - margin && relay_state) {
          relay_state = 0;
          digitalWrite(RELAY_PIN, !RELAY_ACTIVE_LEVEL);
      }
    }
    
    if (eeprom_save_ts && eeprom_save_ts <= ts) eeprom_save();

    #ifdef HDCTL_REPORT_INTERVAL
    static unsigned long tr = 0;
    if (tr <= ts) {
        tr += HDCTL_REPORT_INTERVAL;
        Serial.print("{P<");
        Serial.print(pressure);
        Serial.print(" S<");
        Serial.print(eeprom.setpoint);
        Serial.print(" H<");
        Serial.print(eeprom.hysteresis);
        Serial.print(" R<");
        Serial.print(relay_state);
        Serial.print("}\n");
    }
    #endif
}

bool read_pressure() {
    int sensorValue = analogRead(SENSOR_PIN);
    pressure = 3/35.0*sensorValue + 153/7.0;
    return true;
}

void eeprom_init() {
    int sreg = SREG;
    cli();
    while (!eeprom_is_ready());
    eeprom_read_block((void * ) & eeprom, (void * ) 0, sizeof(eeprom));
    SREG = sreg;

    if (isnan(eeprom.setpoint) || isnan(eeprom.hysteresis)) {
        eeprom.setpoint = 5.0;
        eeprom.hysteresis = 0.1;
        eeprom_save();
    }
}

void eeprom_save() {
    int sreg = SREG;
    cli();
    while (!eeprom_is_ready());
    eeprom_write_block((void * ) & eeprom, (void * ) 0, sizeof(eeprom));
    SREG = sreg;
    eeprom_save_ts = 0;
}

void key_press(byte key) {
    float d = 0;

    switch (key) {
    case KEY_SELECT:
        ++dsp_state;
        dsp_state %= 2;
        dsp.setCursor(0, 1);
        if (dsp_state) dsp.print("SP: "); // setpoint
        else dsp.print("HS: "); // hysteresis
        break;
    case KEY_UP:
        d = 1.0;
        break;
    case KEY_DOWN:
        d = -1.0;
        break;
    case KEY_LEFT:
        d = -0.05;
        break;
    case KEY_RIGHT:
        d = +0.05;
        break;
    default:
        return;
    }

    dsp.setCursor(4, 1);
    if (dsp_state) {
        eeprom.setpoint = min(120, max(1, eeprom.setpoint + d));
        dsp.print(eeprom.setpoint);
    } else {
        eeprom.hysteresis = min(100, max(0, eeprom.hysteresis + d));
        dsp.print(eeprom.hysteresis);
    }
    dsp.print(DSP_CLR);
}

void key_release(byte key) {
    if (key == KEY_SELECT) return; // no changes
    eeprom_save_ts = millis() + 2000; // store changes in 2 sec after last change
}

void key_poll() {
    static byte last_key = KEY_NONE;

    byte key = key_read();
    delayMicroseconds(200);
    if (key != key_read()) return; // debounce
    if (last_key == key) return;
    // state change
    if (key == KEY_NONE) key_release(last_key);
    else key_press(key);
    last_key = key;
}

byte key_read() {
    byte a = analogRead(A0) >> 6;
    //  Serial.println(a);
    if (a == 0xf) return KEY_NONE;
    if (a > 8) return KEY_SELECT;
    if (a > 5) return KEY_LEFT;
    if (a > 1) return KEY_DOWN;
    if (a) return KEY_UP;
    return KEY_RIGHT;
}
