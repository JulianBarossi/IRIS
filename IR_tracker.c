#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

volatile bool gCheckADC = false;

volatile uint16_t sUp = 0;
volatile uint16_t sRight = 0;
volatile uint16_t sDown = 0;
volatile uint16_t sLeft = 0;

//scan settings
#define PAN_MIN_ANGLE      0
#define PAN_MAX_ANGLE      180
#define PAN_STEP_ANGLE     5

#define TILT_MIN_ANGLE     30
#define TILT_MAX_ANGLE     150
#define TILT_STEP_ANGLE    5

#define SIGNAL_THRESHOLD   500U
#define SERVO_SETTLE_CYCLES  800000U

//servo globals
#define PAN_SERVO_ID    1
#define TILT_SERVO_ID   2

#define STS_HEADER      0xFF
#define STS_INST_WRITE  0x03

#define STS_ADDR_GOAL_POSITION  0x2A
#define STS_ADDR_GOAL_TIME      0x2C
#define STS_ADDR_GOAL_SPEED     0x2E

#define SERVO_POS_MIN   0
#define SERVO_POS_MAX   4095

#define SERVO_MOVE_TIME  500
#define SERVO_MOVE_SPEED 500


#define TRACK_TICK_MS           20      // ~50Hz fine loop
#define TRACK_TIMEOUT_TICKS     500     // 10s max in fine tracking
#define ON_TARGET_ERR           40      // raw ADC counts
#define ON_TARGET_HOLD_TICKS    10      
#define LOST_TOTAL_THRESHOLD    300U    

//PID
#define PAN_KP    0.05f
#define PAN_KI    0.001f
#define PAN_KD    0.02f

#define TILT_KP   0.05f
#define TILT_KI   0.001f
#define TILT_KD   0.02f

#define I_CLAMP   200.0f
#define STEP_CLAMP 50    // max servo units to move per tick

static int32_t panPosRaw  = 0;
static int32_t tiltPosRaw = 0;

typedef struct
{
    float kp, ki, kd;
    float integral;
    float prevErr;
} PID;

static PID panPID  = { PAN_KP,  PAN_KI,  PAN_KD,  0.0f, 0.0f };
static PID tiltPID = { TILT_KP, TILT_KI, TILT_KD, 0.0f, 0.0f };


typedef struct
{
    uint16_t bestPanAngle;
    uint16_t bestTiltAngle;
    uint32_t bestSignal;
    int32_t horizontalError;
    int32_t verticalError;
    bool beaconFound;
} ScanResult;


uint16_t angleToSTSPosition(uint16_t angle)
{
    if (angle > 360) angle = 360;
    return (uint16_t)(((uint32_t)angle * SERVO_POS_MAX) / 360U);
}

uint8_t stsChecksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 2; i < length - 1; i++) sum += packet[i];
    return (uint8_t)(~sum);
}

void uartSendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART0)) { }
    DL_UART_Main_transmitData(UART0, data);
}

void uartSendPacket(uint8_t *packet, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) uartSendByte(packet[i]);
}

void stsWritePosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t packet[15];
    packet[0]  = 0xFF;
    packet[1]  = 0xFF;
    packet[2]  = id;
    packet[3]  = 0x09;
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
    panPosRaw = position;
    stsWritePosition(PAN_SERVO_ID, position, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}

void servoMoveTilt(uint16_t angle)
{
    uint16_t position = angleToSTSPosition(angle);
    tiltPosRaw = position;
    stsWritePosition(TILT_SERVO_ID, position, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}

// pan servo with raw units
void servoMovePanRaw(int32_t pos)
{
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    panPosRaw = pos;
    stsWritePosition(PAN_SERVO_ID, (uint16_t)pos, TRACK_TICK_MS, 0);
}

// tilt servo with raw units
void servoMoveTiltRaw(int32_t pos)
{
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    tiltPosRaw = pos;
    stsWritePosition(TILT_SERVO_ID, (uint16_t)pos, TRACK_TICK_MS, 0);
}

void delayServoSettle(void)
{
    delay_cycles(SERVO_SETTLE_CYCLES);
}

// fine track tick, change value if needed
void delayTrackTick(void)
{
    delay_cycles(SERVO_SETTLE_CYCLES);
}


void readFourIRSensors(void)
{
    gCheckADC = false;
    DL_ADC12_startConversion(ADC12_0_INST);
    while (gCheckADC == false) { __WFE(); }

    sUp    = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    sRight = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    sDown  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    sLeft  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);

    DL_ADC12_enableConversions(ADC12_0_INST);
}


ScanResult runScan(void)
{
    ScanResult result = {
        .bestPanAngle = PAN_MIN_ANGLE,
        .bestTiltAngle = TILT_MIN_ANGLE,
        .bestSignal = 0,
        .horizontalError = 0,
        .verticalError = 0,
        .beaconFound = false
    };

    for (uint16_t tilt = TILT_MIN_ANGLE; tilt <= TILT_MAX_ANGLE; tilt += TILT_STEP_ANGLE)
    {
        servoMoveTilt(tilt);
        delayServoSettle();

        for (uint16_t pan = PAN_MIN_ANGLE; pan <= PAN_MAX_ANGLE; pan += PAN_STEP_ANGLE)
        {
            servoMovePan(pan);
            delayServoSettle();

            readFourIRSensors();

            uint32_t totalSignal =
                (4U * 4095U) -
                ((uint32_t)sUp + (uint32_t)sRight + (uint32_t)sDown + (uint32_t)sLeft);

            int32_t horizontalError = (int32_t)sLeft  - (int32_t)sRight;  /* sign flipped */
            int32_t verticalError   = (int32_t)sDown  - (int32_t)sUp;     /* sign flipped */

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

    if (result.bestSignal > SIGNAL_THRESHOLD) result.beaconFound = true;
    return result;
}

// PID
static float pidStep(PID *p, float err, float dt)
{
    p->integral += err * dt;
    if (p->integral >  I_CLAMP) p->integral =  I_CLAMP;
    if (p->integral < -I_CLAMP) p->integral = -I_CLAMP;

    float deriv = (dt > 0.0f) ? (err - p->prevErr) / dt : 0.0f;
    p->prevErr = err;

    return p->kp * err + p->ki * p->integral + p->kd * deriv;
}

static void pidReset(PID *p)
{
    p->integral = 0.0f;
    p->prevErr  = 0.0f;
}

static int32_t clampStep(int32_t v)
{
    if (v >  STEP_CLAMP) return  STEP_CLAMP;
    if (v < -STEP_CLAMP) return -STEP_CLAMP;
    return v;
}

/* Fine-track: drive (right-left) and (up-down) errors to zero with PID.
 * Returns true on lock, false on lost/timeout (caller should re-scan). */
bool fineTrack(void)
{
    pidReset(&panPID);
    pidReset(&tiltPID);

    uint32_t lockedTicks = 0;
    uint32_t totalTicks  = 0;
    const float dt = TRACK_TICK_MS / 1000.0f;

    while (totalTicks < TRACK_TIMEOUT_TICKS)
    {
        readFourIRSensors();

        /* Signal strength (inverted ADC; see note above). */
        uint32_t total =
            (4U * 4095U) -
            ((uint32_t)sUp + (uint32_t)sRight + (uint32_t)sDown + (uint32_t)sLeft);

        if (total < LOST_TOTAL_THRESHOLD)
        {
            return false;  /* lost the beacon, re-scan */
        }

        /* Pair-difference errors. Signs flipped because lower ADC = brighter:
         *   target on the right -> sRight LOWER than sLeft -> (sLeft - sRight) > 0
         *   target above         -> sUp    LOWER than sDown -> (sDown - sUp)   > 0 */
        int32_t panErr  = (int32_t)sLeft - (int32_t)sRight;
        int32_t tiltErr = (int32_t)sDown - (int32_t)sUp;

        /* PID -> raw servo delta */
        int32_t panU  = (int32_t)pidStep(&panPID,  (float)panErr,  dt);
        int32_t tiltU = (int32_t)pidStep(&tiltPID, (float)tiltErr, dt);

        panU  = clampStep(panU);
        tiltU = clampStep(tiltU);

        servoMovePanRaw(panPosRaw   + panU);
        servoMoveTiltRaw(tiltPosRaw + tiltU);

        /* Lock detection with hold time */
        int32_t absPan  = panErr  < 0 ? -panErr  : panErr;
        int32_t absTilt = tiltErr < 0 ? -tiltErr : tiltErr;

        if (absPan < ON_TARGET_ERR && absTilt < ON_TARGET_ERR)
        {
            lockedTicks++;
            if (lockedTicks >= ON_TARGET_HOLD_TICKS) return true;
        }
        else
        {
            lockedTicks = 0;
        }

        delayTrackTick();
        totalTicks++;
    }

    return false;  /* timed out without locking */
}


int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    while (1)
    {
        ScanResult scan = runScan();

        if (!scan.beaconFound)
        {
            /* Nothing seen in the whole sweep -- loop and try again. */
            continue;
        }

        /* Move to the coarse best, then hand off to PID for precise pointing. */
        servoMovePan(scan.bestPanAngle);
        servoMoveTilt(scan.bestTiltAngle);
        delayServoSettle();

        if (fineTrack())
        {
            /* LOCKED. Bearing is in panPosRaw / tiltPosRaw (raw 0..4095).
             * Hook for the rest of the system: flag the fire, log the angle,
             * notify whoever's in charge of the next stage. */

            /* Hold lock with a slower maintenance loop until we lose it. */
            while (fineTrack()) { /* keep tracking */ }
        }
        /* Lost or timed out -> fall through, top of loop re-scans. */
    }
}

/* ADC interrupt */
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
