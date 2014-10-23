/**
 *******************************************************************************
 * @file    application.cpp
 * @authors chrono
 * @version V1.0.0 (codename aziz)
 * @date    2014-10-15
 * @brief   spark-lighter - Robot /w 4ch (RGBW) LED + PIR & Ambient Light Sensor
 *******************************************************************************
  Copyright (c) 2014 Apollo-NG - https://apollo.open-resource.org/

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/// Includes ///////////////////////////////////////////////////////////////////

#include "application.h"


/// Hardware I/O mapping ///////////////////////////////////////////////////////

// Inputs (DYP-ME003 PIR Sensor -> D2 + TEMT6000 Ambient Light Sensor -> A0)
const uint8_t pinPIR    =               2;
const uint8_t pinAMB    =               10;

// Outputs (RGBW Channels -> MOSFET/Gatedriver inputs (A4-A7))
const uint8_t pinR      =               15;
const uint8_t pinG      =               14;
const uint8_t pinB      =               17;
const uint8_t pinW      =               16;


/// Time mapping ///////////////////////////////////////////////////////////////

const uint8_t GPB       =               30; // Grace Period Baselength in Seconds
const uint8_t GPM       =               90; // Maximum Grace Period length in Seconds
uint8_t EGP             =               GPB;// Elastic Grace Period length (dynamic)
uint16_t timeDiff       =               0;

const uint8_t bNight    =               21; // Begin of Night hours
const uint8_t eNight    =               6;  // End of Night hours

uint32_t lastMotion     =               0;
uint32_t lastTimeSync   = millis        ();

const uint16_t PWM_FREQ =               1000;// PWM Frequency in Hertz
uint16_t TIM_ARR        = (uint16_t)    (24000000/PWM_FREQ)-1; // Don't change!

/// Let's try to keep it stateful this time ////////////////////////////////////

uint16_t ambLux         =               0;
uint8_t ledR            =               0;
uint8_t ledG            =               0;
uint8_t ledB            =               0;
uint8_t ledW            =               0;

// States
//  0x1: Online & Ready
//  0x2: PIR Motion triggered
//  0x4: Human Presence
//  0x8: Grace Period
// 0x16: Event Notification
// 0x32: Night

uint8_t state           =               0x0;


/// Function prototypes ////////////////////////////////////////////////////////

int                     setRGBW         (String rgbwInt);

void                    autolight       (int target);
void                    fadeTo          (String rgbwInt);
void                    motion          (void);
void                    readT6K         (void);
void                    setPWM          (uint8_t pin, uint8_t value);


////////////////////////////////////////////////////////////////////////////////
/// Setup //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SYSTEM_MODE                             (AUTOMATIC);

void                    setup           ()
{
    #ifdef DEBUG
    Serial.begin                        (9600)                                  ;
    #endif

    /// Pre-Define port direction & attach Interrupts

    // Set up DYP-ME003 Passive Infrared Sensor
    pinMode                             (pinPIR, INPUT_PULLDOWN)                ;
    attachInterrupt                     (pinPIR, motion, RISING)                ;

    // Set up MOSFET Gate Driver output lines
    pinMode                             (pinR, OUTPUT)                          ;
    pinMode                             (pinG, OUTPUT)                          ;
    pinMode                             (pinB, OUTPUT)                          ;
    pinMode                             (pinW, OUTPUT)                          ;

    /// Close all PWM valves, to keep time of uncontrolled state at minimum. Also
    /// in this context: I had to add 4 pull-down resistors (10k), each between
    /// one of the Core's PWM output pins and the gate driver's input pin or all
    /// hell would break loose (especially when waving my hands over it) :)

    setPWM                              (pinR, 0)                               ;
    setPWM                              (pinG, 0)                               ;
    setPWM                              (pinB, 0)                               ;
    setPWM                              (pinW, 0)                               ;

    /// Expose variables & function through API

    Spark.variable                      ("ledr", &ledR, INT);
    Spark.variable                      ("ledg", &ledG, INT);
    Spark.variable                      ("ledb", &ledB, INT);
    Spark.variable                      ("ledw", &ledW, INT);
    Spark.variable                      ("ambLux", &ambLux, INT)                ;

    Spark.function                      ("setrgbw", setRGBW)                    ;

    // Set Ready-State bit
    state              |=               0x1                                     ;

}

////////////////////////////////////////////////////////////////////////////////
/// MAIN LOOP //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void                    loop            ()
{
    /// Already time to request time synchronization (1h)? /////////////////////

    if                                  (millis() - lastTimeSync > 3600000)
    {
        Spark.syncTime                  ()                                      ;
        lastTimeSync    = millis        ()                                      ;
    }


    /// New motion detection event triggered? //////////////////////////////////

    if                                  ((state & 0x2) == 0x2)
    {
        #ifdef DEBUG
        Serial.println                  ("Motion Detected")                     ;
        #endif

        Spark.publish                   ("motion", NULL, 60, PRIVATE)           ;

        // Clear trigger state bit /////////////////////////////////////////////

        state          &=               ~0x2                                    ;

        // Remember timestamp of this event ////////////////////////////////////
        lastMotion      = millis        ()                                      ;

        // New presence detected (presence state bit not set)? /////////////////

        if                              ((state & 0x4) == 0)
        {
            #ifdef DEBUG
            Serial.println              ("New presence detected")               ;
            #endif

            // Set presence state bit (4) //////////////////////////////////////
            state      |=               0x4                                     ;

            // Let there be light //////////////////////////////////////////////
            autolight                   (1)                                     ;
        }

        // Are we already in grace period?

        else if                         ((state & 0x8) == 0x8)
        {
            // Presence confidence restored, remove grace status
            state      &=               ~0x8                                    ;
            autolight                   (1)                                     ;
        }

        // Honor current presence's movement by increasing time to GP

        else if                         ((state & 0x4) == 0x4)
        {
            if(EGP < GPM)
            {
                #ifdef DEBUG
                Serial.println("Boosting GP +10...");
                #endif

                EGP     =               EGP+10                                  ;
            }
        }

        // Slow down so that the RGB LED state is observable to humans
        delay                           (250)                                   ;

        // Release control of RGB status Led
        RGB.control                     (false)                                 ;
    }


    /// Is there really anyone left present? ///////////////////////////////////

    if                                  ((state & 0x4) == 0x4)
    {
        timeDiff        = (int)         (millis() - lastMotion)/1000            ;

        #ifdef DEBUG
        Serial.print                    ("Last Motion: ")                       ;
        Serial.println                  (timeDiff)                              ;
        #endif

        // Compare last motion time distance for graceful auto powerdown

        if                              (timeDiff > EGP+30)
        {
            // I'm confident no one is any longer present
            #ifdef DEBUG
            Serial.println              ("No one present - Shutting down")      ;
            #endif

            // Unset presence (4) and grace-period (8) status bits
            state      &=               ~0x4                                    ;
            state      &=               ~0x8                                    ;

            // Reset accumulated Elastic Grace Period boni
            EGP         =               GPB                                     ;

            // Let there be darkness
            autolight                   (0)                                     ;
        }
        else if                         (timeDiff > EGP)
        {
            if                          ((state & 0x8) == 0)
            {
                #ifdef DEBUG
                Serial.println          ("Grace Period started - Fading down")  ;
                #endif

                state  |=               0x8                                     ;
                autolight               (2)                                     ;
            }
        }
    }

    readT6K                             ()                                      ;

    #ifdef DEBUG
    Serial.println                      ("------------------------------------");
    Serial.println                      (millis())                              ;
    Serial.print                        (" -> Ambient Light: ")                 ;
    Serial.println                      (ambLux, DEC)                           ;
    Serial.print                        (" -> State: ")                         ;
    Serial.println                      (state)                                 ;
    Serial.print                        (" -> RSSI: ")                          ;
    Serial.println                      (WiFi.RSSI())                           ;
    #endif

    delay                               (500)                                   ;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void                    autolight       (int target)
{
    // Ramp Up Illumination depending on time/environment
    if                                  (target == 1)
    {
        if                              (Time.hour() < 6 || Time.hour() > 21)
        {
            // Night mode
            while                       (ledR < 128)
            {
                ledR++                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode
            while                       (ledW < 255 && ambLux < 250)
            {
                ledW++                                                          ;
                setPWM                  (pinW, ledW)                            ;
                delay                   (20)                                    ;
                readT6K                 ()                                      ;
            }
        }
    }

    // Fade Down a little to notifiy present humans to move
    else if                             (target == 2)
    {
        if (Time.hour() < 6 || Time.hour() > 21)
        {
            // Night mode
            while(ledR > 64)
            {
                ledR--                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode
            while(ledW > 128)
            {
                ledW--                                                          ;
                setPWM                  (pinW, ledW)                            ;
                delay                   (20)                                    ;
            }
        }
    }

    // Fade Down all
    // FIXME: This is still buggy, there has to be some more thought about collisions
    // between autolight and user overrides.
    else
    {
        if (Time.hour() < 6 || Time.hour() > 21)
        {
            // Night mode
            while(ledR > 0)
            {
                ledR--                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode
            while(ledW > 0)
            {
                ledW--                                                          ;
                setPWM                  (pinW, ledW)                            ;
                delay                   (20)                                    ;
            }
        }
    }
}


void                    fadeTo          (long rgbw, int delaytime)
{
    uint8_t completed   =               0x0                                     ;

    #ifdef DEBUG
    Serial.println                      ("Fading to new target")                ;
    #endif

    // Separate colors from combined 32bit RGBA long
    uint8_t newR        =               (rgbw >> 24) & 0xFF                     ;
    uint8_t newG        =               (rgbw >> 16) & 0xFF                     ;
    uint8_t newB        =               (rgbw >>  8) & 0xFF                     ;
    uint8_t newW        = (int)         ((rgbw >> 0) & 0xFF)                    ;

    while                               (completed != 15)
    {
        // Red Channel
        if                              (ledR < newR)
        {
            ledR++                                                              ;
            setPWM                      (pinR, ledR)                            ;
        }
        else if                         (ledR > newR)
        {
            ledR--                                                              ;
            setPWM                      (pinR, ledR)                            ;
        }
        else { completed |= 0x1; }

        // Green Channel
        if                              (ledG < newG)
        {
            ledG++                                                              ;
            setPWM                      (pinG, ledG)                            ;
        }
        else if                         (ledG > newG)
        {
            ledG--                                                              ;
            setPWM                      (pinG, ledG)                            ;
        }
        else { completed |= 0x2; }

        // Blue Channel
        if                              (ledB < newB)
        {
            ledB++                                                              ;
            setPWM                      (pinB, ledB)                            ;
        }
        else if                         (ledB > newB)
        {
            ledB--                                                              ;
            setPWM                      (pinB, ledB)                            ;
        }
        else { completed |= 0x4; }

        // White Channel
        if                              (ledW < newW)
        {
            ledW++                                                              ;
            setPWM                      (pinW, ledW)                            ;
        }
        else if                         (ledW > newW)
        {
            ledW--                                                              ;
            setPWM                      (pinW, ledW)                            ;
        }
        else { completed |= 0x8; }

        delay                           (delaytime)                             ;
    }

    #ifdef DEBUG
    Serial.print                        ("R: ")                                 ;
    Serial.println                      (ledR)                                  ;
    Serial.print                        ("G: ")                                 ;
    Serial.println                      (ledG)                                  ;
    Serial.print                        ("B: ")                                 ;
    Serial.println                      (ledB)                                  ;
    Serial.print                        ("W: ")                                 ;
    Serial.println                      (ledW)                                  ;
    #endif
}

void                    readT6K         (void)
{
    uint16_t D          = analogRead    (pinAMB)                                ;
    /*
    float U             =               D * 3.3 / 4095.0                        ;
    float I             =               U / 10000.0                             ;
    uint16_t lux        =               I * 1000000 * 2                         ;
    */
    ambLux              =               D * 0.161172161172                      ;
}

void                    motion          (void)
{
    RGB.control                         (true)                                  ;
    RGB.color                           (30, 255, 5)                            ;
    state              |=               0x2                                     ;
}

int                     setRGBW         (String rgbwInt)
{
    #ifdef DEBUG
    Serial.print                        ("setRGBW Called: ")                    ;
    Serial.println                      (rgbwInt)                               ;
    #endif
    fadeTo                              (rgbwInt.toInt(), 20)                   ;
    return                              1                                       ;
}

/*
I needed more control over the PWM than analogWrite() could offer. The 500Hz
wasn't enough for my eyes to appear as flicker free. I'm running with 1kHz now
and so far I cannot really tell anymore. There is residual doubt sometimes, when
I move my head and accelerate my eyeballs from one corner of the vision field into
the opposite one, both in the same direction.

Maybe it's time to try to describe the effect a bit more in detail

https://gist.github.com/technobly/8313449 Thanks @ (Technobly/BDub)
*/

void                    setPWM          (uint8_t pin, uint8_t value)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    if (pin >= TOTAL_PINS || PIN_MAP[pin].timer_peripheral == NULL)
    {
        return;
    }

    // SPI safety check
    if (SPI.isEnabled() == true && (pin == SCK || pin == MOSI || pin == MISO))
    {
        return;
    }

    // I2C safety check
    if (Wire.isEnabled() == true && (pin == SCL || pin == SDA))
    {
        return;
    }

    // Serial1 safety check
    if (Serial1.isEnabled() == true && (pin == RX || pin == TX))
    {
        return;
    }

    if (PIN_MAP[pin].pin_mode != OUTPUT &&
        PIN_MAP[pin].pin_mode != AF_OUTPUT_PUSHPULL)
    {
        return;
    }

    // Don't re-init PWM and cause a glitch if already setup,
    // just update duty cycle and return.
    if (PIN_MAP[pin].pin_mode == AF_OUTPUT_PUSHPULL)
    {
        TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM_ARR + 1) / 255);

        if (PIN_MAP[pin].timer_ch == TIM_Channel_1)
        {
            PIN_MAP[pin].timer_peripheral-> CCR1 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_2)
        {
            PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_3)
        {
            PIN_MAP[pin].timer_peripheral-> CCR3 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_4)
        {
            PIN_MAP[pin].timer_peripheral-> CCR4 = TIM_OCInitStructure.TIM_Pulse;
        }
        return;
    }

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // PWM Frequency : PWM_FREQ (Hz)
    // TIM Counter clock = 24MHz
    uint16_t TIM_Prescaler = (uint16_t)(SystemCoreClock / 24000000) - 1;

    // TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
    uint16_t TIM_CCR = (uint16_t)(value * (TIM_ARR + 1) / 255);

    // AFIO clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    pinMode(pin, AF_OUTPUT_PUSHPULL);

    // TIM clock enable
    if (PIN_MAP[pin].timer_peripheral == TIM2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }
    else if (PIN_MAP[pin].timer_peripheral == TIM3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }
    else if (PIN_MAP[pin].timer_peripheral == TIM4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(PIN_MAP[pin].timer_peripheral, & TIM_TimeBaseStructure);

    // PWM1 Mode configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = TIM_CCR;

    if (PIN_MAP[pin].timer_ch == TIM_Channel_1)
    {
        // PWM1 Mode configuration: Channel1
        TIM_OC1Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC1PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_2)
    {
        // PWM1 Mode configuration: Channel2
        TIM_OC2Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC2PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_3)
    {
        // PWM1 Mode configuration: Channel3
        TIM_OC3Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC3PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_4)
    {
        // PWM1 Mode configuration: Channel4
        TIM_OC4Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC4PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(PIN_MAP[pin].timer_peripheral, ENABLE);

    // TIM enable counter
    TIM_Cmd(PIN_MAP[pin].timer_peripheral, ENABLE);
}
