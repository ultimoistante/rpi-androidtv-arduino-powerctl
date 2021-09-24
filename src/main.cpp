

/** RaspberryPi/AndroidTV Arduino Power Control device
 **
 ** This device implements power control features for RaspberryPi4 AndroidTV mediacenter.
 ** Its main feature is to control power supply to RaspberryPi through a relay.
 ** Communicates with RaspberryPi via UART connection, sending commands (i.e.: shutdown).
 ** It also controls case fan throttling reading RaspberryPi CPU temperature using a thermistor.
 **
 ** Author: Salvatore Carotenuto of OpenMakersItaly
 **
 ** Credits:
 **   Temperature reading code taken from: https://learn.adafruit.com/thermistor/using-a-thermistor
 **/

 // -----------------------------------------------------------------------------
 // -----------------------------------------------------------------------------
 //
 // Arduino Nano connections
 //                                       +-----+
 //                                 +-----|=====|-----+
 //                      D13 [PB5]  |o    |=====|    o|  [PB4] D12
 //                            3v3  |o    +-----+    o| ~[PB3] D11 ---> (standby) status LED (red)
 //                           Vref  |o               o| ~[PB2] D10 ---> (power on) status LED (green)
 //       thermistor <--- A0 [PC0]  |o               o| ~[PB1] D9  ---> serial activity LED
 //                       A1 [PC1]  |o               o|  [PB0] D8  ---> power button
 //                       A2 [PC2]  |o               o|  [PD7] D7  
 //                       A3 [PC3]  |o    Arduino    o| ~[PD6] D6
 //  debug serial RX <--- A4 [PC4]  |o      Nano     o| ~[PD5] D5  ---> fan speed throttle
 //  debug serial TX <--- A5 [PC5]  |o               o|  [PD4] D4  
 //                      A6 [ADC6]  |o               o| ~[PD3] D3  
 //                      A7 [ADC7]  |o               o|  [PD2] D2  ---> RaspberryPi power relay control
 //                             5V  |o               o|  GND
 //                      RST [PC6]  |o               o|  [PC6] RST
 //                            GND  |o     . . .     o|  [PD0] RX  ---> to RaspberryPi TXD (pin  8 - GPIO14)
 //                            Vin  |o     . . .     o|  [PD1] TX  ---> to RaspberryPi RXD (pin 10 - GPIO15)
 //                                 +-----------------+
 //
 // -----------------------------------------------------------------------------
 // -----------------------------------------------------------------------------


#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AceButton.h>
using namespace ace_button;
#include <jled.h>


// -----------------------------------------------------------------------------


#define THERMISTOR_PIN                A0
#define RELAY_PIN                     2
#define STATUS_LED_RED_PIN            11
#define STATUS_LED_GREEN_PIN          10
#define UART_ACT_LED_PIN              9
#define POWER_BUTTON_PIN              8
#define FAN_THROTTLE_PIN              5
#define SW_SERIAL_RX_PIN              A4
#define SW_SERIAL_TX_PIN              A5

#define THERMISTOR_SERIES_RESISTOR    10000 // the value of the 'other' resistor in voltage divider
#define THERMISTOR_NOMINAL_R          10000      // nominal thermistor resistance at 25 degrees C
#define THERMISTOR_NOMINAL_TEMP       25   // thermistor nominal resistance temperature (almost always 25 C)
#define THERMISTOR_BETA_COEFFICIENT   3950 // thermistor beta coefficient (usually 3000-4000)
#define TEMPERATURE_READ_SAMPLES      5 // how many samples to take and average, more takes longer, but is more 'smooth'

#define HW_SERIAL_BAUDRATE            115200
#define SW_SERIAL_BAUDRATE            57600

#define READ_TEMPERATURE_INTERVAL     1000

float currentTemperature = 0;
unsigned long next_read_temperature_time = 0;

#define STATUS_STANDBY                  0
#define STATUS_BOOTING                  1
#define STATUS_ON                       2
#define STATUS_SHUTDOWN                 3
unsigned char currentStatus = STATUS_STANDBY;

// debug (software) serial
SoftwareSerial debugSerial(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN); // RX, TX

// status leds
auto standbyStatusLed = JLed(STATUS_LED_RED_PIN);
auto poweronStatusLed = JLed(STATUS_LED_GREEN_PIN);

// power button
AceButton powerButton(POWER_BUTTON_PIN);


// ----------------------------------------------------------------------------


// Interrupt handler called once a millisecond
SIGNAL(TIMER0_COMPA_vect)
    {
    standbyStatusLed.Update();
    poweronStatusLed.Update();
    //
    powerButton.check();
    }


// ----------------------------------------------------------------------------


float readTemperature()
    {
    float average = 0.0;
    // take N samples in a row, with a slight delay
    for (uint8_t i = 0; i < TEMPERATURE_READ_SAMPLES; i++)
        {
        average += analogRead(THERMISTOR_PIN);
        delay(10);
        }
    average /= TEMPERATURE_READ_SAMPLES;

    // convert the value to resistance
    average = 1023 / average - 1;
    average = THERMISTOR_SERIES_RESISTOR / average;

    float steinhart;
    steinhart = average / THERMISTOR_NOMINAL_R;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= THERMISTOR_BETA_COEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (THERMISTOR_NOMINAL_TEMP + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C
    //
    return steinhart;
    }



// ----------------------------------------------------------------------------


// computes fan pwm value, based on temperature read.
// Mapping inspired by arduino "map" function:
//    long map(long x, long in_min, long in_max, long out_min, long out_max) {
//        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//    }
// 
int computeFanPwm(float temperature)
    {
    int pwmValue = 0;
    float minTemp = 30.0;
    float maxTemp = 60.0;
    int minPwm = 16;
    int maxPwm = 255;
    if (temperature >= maxTemp)
        return maxPwm;
    else
        if (temperature > minTemp && temperature < maxTemp)
            {
            float mappedValue = (temperature - minTemp) * (maxPwm - minPwm) / (maxTemp - minTemp) + minPwm;
            pwmValue = int(mappedValue);
            }
    //
    return pwmValue;
    }


// ----------------------------------------------------------------------------


void updateCurrentStatus(unsigned char newStatus)
    {
    currentStatus = newStatus;
    switch (currentStatus)
        {
            case STATUS_STANDBY:
                standbyStatusLed.Breathe(5000).Forever();
                poweronStatusLed.Off();
                digitalWrite(RELAY_PIN, LOW);
                break;
            case STATUS_BOOTING:
                standbyStatusLed.Blink(25, 250).Forever();
                poweronStatusLed.Blink(25, 250).Forever();
                digitalWrite(RELAY_PIN, HIGH);
                break;
            case STATUS_ON:
                standbyStatusLed.Off();
                poweronStatusLed.Breathe(2000).Forever();
                break;
            case STATUS_SHUTDOWN:
                break;
        }
    }


// -----------------------------------------------------------------------------


void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/)
    {
    switch (eventType)
        {
        // handles "released" event as a "pressed" event, to distiguish it from a "long pressed" event
            case AceButton::kEventReleased:
                Serial.println("button pressed");
                // standbyStatusLed.Off();
                // poweronStatusLed.Breathe(5000).Forever();
                if (currentStatus == STATUS_STANDBY)
                    updateCurrentStatus(STATUS_BOOTING);
                else
                    updateCurrentStatus(STATUS_ON);


                // if (currentWorkingMode == WORK_MODE_NORMAL)
                //   currentWorkingMode = WORK_MODE_ENTER_LEARNING;
                // else if (currentWorkingMode == WORK_MODE_FACTORY_RESET)
                //   currentWorkingMode = WORK_MODE_FACTORY_RESET_CONFIRMED;
                break;
                // 
            // handles "long pressed" event
            case AceButton::kEventLongPressed:
                // if (currentWorkingMode == WORK_MODE_NORMAL)
                //   currentWorkingMode = WORK_MODE_FACTORY_RESET;
                break;
        }
    }


// ----------------------------------------------------------------------------


void setup()
    {
    // for more stable readings, AREF pin is connected to 3.3V, so we set analogReference to EXTERNAL
    analogReference(EXTERNAL);
    //
    // sets USER_BUTTON_PIN as input, with pullup
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
    //
    // sets FAN_THROTTLE_PIN to output
    pinMode(FAN_THROTTLE_PIN, OUTPUT);
    //
    // sets RELAY_PIN mode to output, and low
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    //
    // enables interrupt on "Compare A" function of Timer0 (already used for millis())
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    //
    // user button handler configuration
    ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    //
    // starts hardware serial (connected to RaspberryPi serial)
    Serial.begin(HW_SERIAL_BAUDRATE);
    // starts software (debug) serial
    debugSerial.begin(SW_SERIAL_BAUDRATE);
    //
    // initializes device status
    updateCurrentStatus(STATUS_STANDBY);
    }


// ----------------------------------------------------------------------------


unsigned long next_serialout_time = 0;

void loop()
    {
    unsigned long now = millis();

    if (now >= next_serialout_time)
        {
        debugSerial.println(now);
        next_serialout_time = now + 1000;
        }



    if (now >= next_read_temperature_time)
        {
        currentTemperature = readTemperature();
        Serial.print("Temperature ");
        Serial.print(currentTemperature);
        Serial.println(" *C");
        //
        int pwmValue = computeFanPwm(currentTemperature);
        analogWrite(FAN_THROTTLE_PIN, pwmValue);
        Serial.print("pwmValue ");
        Serial.println(pwmValue);
        //
        next_read_temperature_time = millis() + READ_TEMPERATURE_INTERVAL;
        }

    }


// ----------------------------------------------------------------------------
