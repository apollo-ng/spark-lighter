/**
 *******************************************************************************
 * @file    application.cpp
 * @authors chrono
 * @version V1.0.0 (codename Aziz 3.0)
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

#define VERBOSE

////////////////////////////////////////////////////////////////////////////////
/// Includes ///////////////////////////////////////////////////////////////////

#include "application.h"


////////////////////////////////////////////////////////////////////////////////
/// Hardware I/O mapping ///////////////////////////////////////////////////////

// Inputs (DYP-ME003 PIR Sensor -> D2 & TEMT6000 Ambient Light Sensor -> A0) ///

const uint8_t pinPIR    =               2                                       ;
const uint8_t pinAMB    =               10                                      ;

// Outputs (RGBW Channels -> [A4:A7] -> MOSFET/Gatedriver inputs ) /////////////

const uint8_t pinR      =               15                                      ;
const uint8_t pinG      =               14                                      ;
const uint8_t pinB      =               17                                      ;
const uint8_t pinW      =               16                                      ;


////////////////////////////////////////////////////////////////////////////////
/// Time mapping ///////////////////////////////////////////////////////////////

const uint8_t GPB       =               30; // Grace Period Baselength in Seconds
const uint8_t GPM       =               90; // Maximum Grace Period length in Seconds
const uint8_t bNight    =               21; // Begin of Night hours
const uint8_t eNight    =               6;  // End of Night hours

uint8_t EGP             =               GPB;// Elastic Grace Period length (dynamic)
uint16_t timeDiff       =               0;
uint32_t lastMotion     =               0;
uint32_t lastTimeSync   = millis        ();


////////////////////////////////////////////////////////////////////////////////
/// States /////////////////////////////////////////////////////////////////////

// Numeric /////////////////////////////////////////////////////////////////////

uint16_t ambLux         =               0                                       ;
uint8_t ledR            =               0                                       ;
uint8_t ledG            =               0                                       ;
uint8_t ledB            =               0                                       ;
uint8_t ledW            =               0                                       ;

// Bitwise /////////////////////////////////////////////////////////////////////

uint8_t state           =               0x0                                     ;

/* State Table ****************************************************************/
/*
   0x1                  :               Online & Ready
   0x2                  :               PIR Motion triggered
   0x4                  :               Human Presence
   0x8                  :               Grace Period
   0x16                 :               Event Notification
   0x32                 :               Night
*/


////////////////////////////////////////////////////////////////////////////////
/// Function prototypes ////////////////////////////////////////////////////////

int                     setRGBW         (String rgbwInt)                        ;
void                    setPWM          (uint8_t pin, uint8_t value)            ;
void                    fadeTo          (long rgbw, int delaytime)              ;
void                    autolight       (int target)                            ;
void                    motionISR       (void)                                  ;
uint16_t                readT6K         (void)                                  ;



////////////////////////////////////////////////////////////////////////////////
/// Setup //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SYSTEM_MODE                             (AUTOMATIC)                             ;

void                    setup           ()
{
    #ifdef VERBOSE
    Serial.begin                        (9600)                                  ;
    #endif

    ////////////////////////////////////////////////////////////////////////////
    /// Pre-Define port direction & attach Interrupts //////////////////////////

    // Set up DYP-ME003 Passive Infrared Sensor ////////////////////////////////

    pinMode                             (pinPIR, INPUT_PULLDOWN)                ;
    attachInterrupt                     (pinPIR, motionISR, RISING)             ;

    // Set up MOSFET Gate Driver output lines //////////////////////////////////

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

    ////////////////////////////////////////////////////////////////////////////
    /// Expose variables & function through spark-server API ///////////////////

    Spark.variable                      ("ledr", &ledR, INT)                    ;
    Spark.variable                      ("ledg", &ledG, INT)                    ;
    Spark.variable                      ("ledb", &ledB, INT)                    ;
    Spark.variable                      ("ledw", &ledW, INT)                    ;
    Spark.variable                      ("ambLux", &ambLux, INT)                ;
    Spark.function                      ("setrgbw", setRGBW)                    ;

    ////////////////////////////////////////////////////////////////////////////
    /// Set Ready-State bit ////////////////////////////////////////////////////

    state              |=               0x1                                     ;
}



////////////////////////////////////////////////////////////////////////////////
/// MAIN LOOP //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void                    loop            ()
{
    ////////////////////////////////////////////////////////////////////////////
    /// Already time to request time synchronization (1h)? /////////////////////

    if                                  (millis() - lastTimeSync > 3600000)
    {
        Spark.syncTime                  ()                                      ;
        lastTimeSync    = millis        ()                                      ;
    }


    ////////////////////////////////////////////////////////////////////////////
    /// Is the motion trigger state bit set (new motion detected)? /////////////

    if                                  ((state & 0x2) == 0x2)
    {
        #ifdef VERBOSE
        Serial.println                  ("Motion Detected")                     ;
        #endif

        // Clear motion trigger state bit //////////////////////////////////////

        state          &=               ~0x2                                    ;

        // Publish our motion event through our spark-server's event firehose //

        Spark.publish                   ("motion", NULL, 60, PRIVATE)           ;

        // Remember the timestamp of this event ////////////////////////////////

        lastMotion      = millis        ()                                      ;

        // New presence detected (presence state bit not set)? /////////////////

        if                              ((state & 0x4) == 0)
        {
            #ifdef VERBOSE
            Serial.println              ("New presence detected")               ;
            #endif

            // Set presence state bit (4) //////////////////////////////////////

            state      |=               0x4                                     ;

            // Let there be light //////////////////////////////////////////////

            autolight                   (1)                                     ;
        }

        // Are we already in grace period? /////////////////////////////////////

        else if                         ((state & 0x8) == 0x8)
        {
            // Presence confidence restored, remove grace status bit ///////////

            state      &=               ~0x8                                    ;
            autolight                   (1)                                     ;
        }

        // Honor current presence's movement by increasing time to GP //////////

        else if                         ((state & 0x4) == 0x4)
        {
            if(EGP < GPM)
            {
                #ifdef VERBOSE
                Serial.println("Boosting GP +10...");
                #endif

                EGP     =               EGP+10                                  ;
            }
        }

        // Slow down so that the RGB LED state is observable to humans /////////

        delay                           (250)                                   ;

        // Release control of RGB status Led ///////////////////////////////////

        RGB.control                     (false)                                 ;
    }


    ////////////////////////////////////////////////////////////////////////////
    /// Is there really anyone left present? ///////////////////////////////////

    if                                  ((state & 0x4) == 0x4)
    {
        timeDiff        = (int)         (millis() - lastMotion)/1000            ;

        #ifdef VERBOSE
        Serial.print                    ("Last Motion: ")                       ;
        Serial.println                  (timeDiff)                              ;
        #endif

        // Compare last motion time distance for graceful auto powerdown ///////

        if                              (timeDiff > EGP+30)
        {
            // I'm confident no one is any longer present //////////////////////

            #ifdef VERBOSE
            Serial.println              ("No one present - Shutting down")      ;
            #endif

            // Unset presence (4) and grace-period (8) status bits /////////////

            state      &=               ~0x4                                    ;
            state      &=               ~0x8                                    ;

            // Reset accumulated Elastic Grace Period boni /////////////////////

            EGP         =               GPB                                     ;

            // Let there be darkness ///////////////////////////////////////////

            autolight                   (0)                                     ;
        }

        else if                         (timeDiff > EGP)
        {
            if                          ((state & 0x8) == 0)
            {
                #ifdef VERBOSE
                Serial.println          ("Grace Period started - Fading down")  ;
                #endif

                // Set grace period state bit (8) //////////////////////////////

                state  |=               0x8                                     ;

                // Fade the light down a little to remind the human to move ////
                autolight               (2)                                     ;
            }
        }
    }

    ambLux              = readT6K       ()                                      ;

    #ifdef VERBOSE
    Serial.println                      ("------------------------------------");
    Serial.println                      (millis())                              ;
    Serial.print                        (" -> Ambient Light: ")                 ;
    Serial.println                      (ambLux, DEC)                           ;
    Serial.print                        (" -> State: ")                         ;
    Serial.println                      (state)                                 ;
    Serial.print                        (" -> RSSI: ")                          ;
    Serial.println                      (WiFi.RSSI())                           ;
    #endif

    delay                               (150)                                   ;
}

/// END MAIN LOOP //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void                    autolight       (int target)
{
    if                                  (target == 1)
    {
        ////////////////////////////////////////////////////////////////////////
        // Ramp up to Maximum, depending on time/environment ///////////////////

        if                              (  Time.hour() < eNight
                                        || Time.hour() > bNight)
        {
            // Night mode //////////////////////////////////////////////////////

            while                       (ledR < 128)
            {
                ledR++                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode ////////////////////////////////////////////////////////

            while                       (ledW < 255 && ambLux < 250)
            {
                ledW++                                                          ;
                setPWM                  (pinW, ledW)                            ;
                delay                   (20)                                    ;
                ambLux  = readT6K       ()                                      ;
            }
        }
    }

    else if                             (target == 2)
    {
        ////////////////////////////////////////////////////////////////////////
        // Fade Down a little to notifiy present humans to move ////////////////

        if                              (  Time.hour() < eNight
                                        || Time.hour() > bNight)
        {
            // Night mode //////////////////////////////////////////////////////

            while                       (ledR > 64)
            {
                ledR--                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode ////////////////////////////////////////////////////////

            while                       (ledW > 128)
            {
                ledW--                                                          ;
                setPWM                  (pinW, ledW)                            ;
                delay                   (20)                                    ;
            }
        }
    }

    else
    {
        ////////////////////////////////////////////////////////////////////////
        // Fade Down all
        // FIXME: This is still buggy, there has to be some more thought about
        // collisions between autolight and user overrides.

        if                              (  Time.hour() < eNight
                                        || Time.hour() > bNight)
        {
            // Night mode //////////////////////////////////////////////////////

            while                       (ledR > 0)
            {
                ledR--                                                          ;
                setPWM                  (pinR, ledR)                            ;
                delay                   (20)                                    ;
            }
        }
        else
        {
            // Day mode ////////////////////////////////////////////////////////

            while                       (ledW > 0)
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

    #ifdef VERBOSE
    Serial.println                      ("Fading to new target")                ;
    #endif

    // Separate colors from combined 32bit RGBA long ///////////////////////////

    uint8_t newR        =               (rgbw >> 24) & 0xFF                     ;
    uint8_t newG        =               (rgbw >> 16) & 0xFF                     ;
    uint8_t newB        =               (rgbw >>  8) & 0xFF                     ;
    uint8_t newW        = (int)         ((rgbw >> 0) & 0xFF)                    ;

    while                               (completed != 15)
    {
        // Red Channel /////////////////////////////////////////////////////////

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

        // Green Channel ///////////////////////////////////////////////////////

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

        // Blue Channel ////////////////////////////////////////////////////////

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

        // White Channel ///////////////////////////////////////////////////////

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

    #ifdef VERBOSE
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

uint16_t                readT6K         (void)
{
    uint16_t D          = analogRead    (pinAMB)                                ;
    /*
    float U             =               D * 3.3 / 4095.0                        ;
    float I             =               U / 10000.0                             ;
    uint16_t lux        =               I * 1000000 * 2                         ;
    */
    return                              (D * 0.161172)                          ;
}

void                    motionISR       (void)
{
    RGB.control                         (true)                                  ;
    RGB.color                           (30, 255, 5)                            ;
    state              |=               0x2                                     ;
}

int                     setRGBW         (String rgbwInt)
{
    #ifdef VERBOSE
    Serial.print                        ("setRGBW Called: ")                    ;
    Serial.println                      (rgbwInt)                               ;
    #endif
    fadeTo                              (rgbwInt.toInt(), 20)                   ;
    return                              1                                       ;
}
