/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/*
   ADC Globals
*/

volatile bool gCheckADC = false;

volatile uint16_t sUp = 0;
volatile uint16_t sRight = 0;
volatile uint16_t sDown = 0;
volatile uint16_t sLeft = 0;

/*
   Scan Settings
   Adjust these for your hardware
*/

#define PAN_MIN_ANGLE      0
#define PAN_MAX_ANGLE      180
#define PAN_STEP_ANGLE     5

#define TILT_MIN_ANGLE     30
#define TILT_MAX_ANGLE     150
#define TILT_STEP_ANGLE    5

#define SIGNAL_THRESHOLD   500U
#define SERVO_SETTLE_CYCLES  800000U

/*
    Servo Globals
*/

// SERVO ID's
#define PAN_SERVO_ID    1
#define TILT_SERVO_ID   2

//SERVO HEADER TO START CODE
#define STS_HEADER      0xFF
#define STS_INST_WRITE  0x03
                                // registers
#define STS_ADDR_GOAL_POSITION  0x2A // where the servo should move
#define STS_ADDR_GOAL_TIME      0x2C // how long movement should take
#define STS_ADDR_GOAL_SPEED     0x2E // motor speed limit

// 12-bit position values
/*
    Angle       Position
    0           0
    180         ~2047
    360         4095
*/
#define SERVO_POS_MIN   0
#define SERVO_POS_MAX   4095

#define SERVO_MOVE_TIME  500// ms
#define SERVO_MOVE_SPEED 500    // lIMITS MOTOR SPED


/*
   Scan Result
 */

typedef struct
{
    uint16_t bestPanAngle;
    uint16_t bestTiltAngle;
    uint32_t bestSignal;
    int32_t horizontalError; 
    int32_t verticalError;
    bool beaconFound;
} ScanResult;

/*
   Servo placeholders
   Replace with your team's servo code
*/

uint16_t angleToSTSPosition(uint16_t angle)
{
    if (angle > 360)
    {
        angle = 360;
    }

    return (uint16_t)(((uint32_t)angle * SERVO_POS_MAX) / 360U);
}

uint8_t stsChecksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;

    // Skip 0xFF 0xFF header
    for (uint8_t i = 2; i < length - 1; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}

void uartSendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART0))
    {
    }

    DL_UART_Main_transmitData(UART0, data);
}

void uartSendPacket(uint8_t *packet, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        uartSendByte(packet[i]);
    }
}

void stsWritePosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t packet[15];

    packet[0]  = 0xFF;
    packet[1]  = 0xFF;
    packet[2]  = id;
    packet[3]  = 0x09;              // length
    packet[4]  = STS_INST_WRITE;
    packet[5]  = STS_ADDR_GOAL_POSITION;

    packet[6]  = position & 0xFF;
    packet[7]  = (position >> 8) & 0xFF;

    packet[8]  = time & 0xFF;
    packet[9]  = (time >> 8) & 0xFF;

    packet[10] = speed & 0xFF;
    packet[11] = (speed >> 8) & 0xFF;

    packet[12] = stsChecksum(packet, 13);

    uartSendPacket(packet, 13);
}

void servoMovePan(uint16_t angle)
{
    uint16_t position = angleToSTSPosition(angle);
    stsWritePosition(PAN_SERVO_ID, position, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}

void servoMoveTilt(uint16_t angle)
{
    uint16_t position = angleToSTSPosition(angle);
    stsWritePosition(TILT_SERVO_ID, position, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}
// void delayServoSettle(void)
// {
//     delay_cycles(SERVO_SETTLE_CYCLES);
// }

/* 
   ADC Read Function
   Requires SysConfig:
   MEM0 = UP
   MEM1 = RIGHT
   MEM2 = DOWN
   MEM3 = LEFT
   Interrupt = MEM3 result loaded
*/

void readFourIRSensors(void)
{
    gCheckADC = false;

    DL_ADC12_startConversion(ADC12_0_INST);

    while (gCheckADC == false)
    {
        __WFE();
    }

    sUp    = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    sRight = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    sDown  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    sLeft  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);

    DL_ADC12_enableConversions(ADC12_0_INST);
}


ScanResult runScan(void)
{
    ScanResult result;

    result.bestPanAngle = PAN_MIN_ANGLE;
    result.bestTiltAngle = TILT_MIN_ANGLE;
    result.bestSignal = 0;
    result.horizontalError = 0;
    result.verticalError = 0;
    result.beaconFound = false;

    for (uint16_t tilt = TILT_MIN_ANGLE; tilt <= TILT_MAX_ANGLE; tilt += TILT_STEP_ANGLE)
    {
        servoMoveTilt(tilt);
        delayServoSettle();

        for (uint16_t pan = PAN_MIN_ANGLE; pan <= PAN_MAX_ANGLE; pan += PAN_STEP_ANGLE)
        {
            servoMovePan(pan); // Need to implement this
            delayServoSettle(); // Need to implement this

            readFourIRSensors();

            uint32_t totalSignal =
                (uint32_t)sUp +
                (uint32_t)sRight +
                (uint32_t)sDown +
                (uint32_t)sLeft;

            int32_t horizontalError = (int32_t)sRight - (int32_t)sLeft;
            int32_t verticalError   = (int32_t)sUp - (int32_t)sDown;

            if (totalSignal > result.bestSignal)
            {
                result.bestSignal = totalSignal;
                result.bestPanAngle = pan;
                result.bestTiltAngle = tilt;
                result.horizontalError = horizontalError;
                result.verticalError = verticalError;
            }
        }
    }

    if (result.bestSignal > SIGNAL_THRESHOLD)
    {
        result.beaconFound = true;
    }

    return result;
}



int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    while (1)
    {
        ScanResult scan = runScan();

        if (scan.beaconFound)
        {


            servoMovePan(scan.bestPanAngle);
            servoMoveTilt(scan.bestTiltAngle);
        }
        else
        {
            // If beacon not found, scan again
        }
    }
}

// ADC interrupt

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST))
    {
        case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
            gCheckADC = true;
            break;

        default:
            break;
    }
}