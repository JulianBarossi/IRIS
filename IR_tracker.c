#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

volatile bool gCheckADC = false;

volatile uint16_t sUp = 0;
volatile uint16_t sRight = 0;
volatile uint16_t sDown = 0;
volatile uint16_t sLeft = 0;


#define PAN_POS_MIN     1024
#define PAN_POS_MAX     3072
#define PAN_POS_CENTER  2048

#define TILT_POS_MIN    2100   // vertical (looking up)
#define TILT_POS_MAX    3000   // horizontal (looking forward)
#define TILT_POS_CENTER 2550   // midway

// approximately 10 degrees
#define PAN_STEP        120
#define TILT_STEP       120

/* Scan settling: how long to wait after commanding a move before sampling.
 * Tune until the servo is consistently arrived before the ADC reads. */
#define SERVO_SETTLE_CYCLES  5000000U

/* Threshold for "we saw something" at coarse-scan resolution */
#define SIGNAL_THRESHOLD     500U


// servo protocol
#define PAN_SERVO_ID    1
#define TILT_SERVO_ID   2

#define STS_HEADER      0xFF
#define STS_INST_WRITE  0x03
#define STS_ADDR_GOAL_POSITION  0x2A

#define SERVO_MOVE_TIME  100000   // ms per move during coarse scan
#define SERVO_MOVE_SPEED 0      // 0, let time dictate movement

// pid
#define TRACK_TICK_MS           20
#define TRACK_TIMEOUT_TICKS     500
#define ON_TARGET_ERR           40
#define ON_TARGET_HOLD_TICKS    10
#define LOST_TOTAL_THRESHOLD    200U

#define PAN_KP    0.05f
#define PAN_KI    0.001f
#define PAN_KD    0.02f

#define TILT_KP   0.05f
#define TILT_KI   0.001f
#define TILT_KD   0.02f

#define I_CLAMP    200.0f
#define STEP_CLAMP 50


static int32_t panPosRaw  = PAN_POS_CENTER;
static int32_t tiltPosRaw = TILT_POS_CENTER;

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
    uint16_t bestPanPos;       
    uint16_t bestTiltPos;
    uint32_t bestSignal;
    int32_t  horizontalError;
    int32_t  verticalError;
    bool     beaconFound;
} ScanResult;


//uart and packet functions
uint8_t stsChecksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 2; i < length - 1; i++) sum += packet[i];
    return (uint8_t)(~sum);
}

void uartSendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART1)) { }
    DL_UART_Main_transmitData(UART1, data);
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

// make sure servo position is within bounds
static int32_t clampPan(int32_t pos)
{
    if (pos < PAN_POS_MIN) return PAN_POS_MIN;
    if (pos > PAN_POS_MAX) return PAN_POS_MAX;
    return pos;
}

static int32_t clampTilt(int32_t pos)
{
    if (pos < TILT_POS_MIN) return TILT_POS_MIN;
    if (pos > TILT_POS_MAX) return TILT_POS_MAX;
    return pos;
}

// move servo
void servoMovePan(int32_t pos)
{
    pos = clampPan(pos);
    panPosRaw = pos;
    stsWritePosition(PAN_SERVO_ID, (uint16_t)pos, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}

void servoMoveTilt(int32_t pos)
{
    pos = clampTilt(pos);
    tiltPosRaw = pos;
    stsWritePosition(TILT_SERVO_ID, (uint16_t)pos, SERVO_MOVE_TIME, SERVO_MOVE_SPEED);
}

// shorter move time for fine tracking
void servoMovePanRaw(int32_t pos)
{
    pos = clampPan(pos);
    panPosRaw = pos;
    stsWritePosition(PAN_SERVO_ID, (uint16_t)pos, TRACK_TICK_MS, SERVO_MOVE_SPEED);
}

void servoMoveTiltRaw(int32_t pos)
{
    pos = clampTilt(pos);
    tiltPosRaw = pos;
    stsWritePosition(TILT_SERVO_ID, (uint16_t)pos, TRACK_TICK_MS, SERVO_MOVE_SPEED);
}

void delayServoSettle(void) { delay_cycles(SERVO_SETTLE_CYCLES); }
void delayTrackTick(void)   { delay_cycles(SERVO_SETTLE_CYCLES); }


// ADC
void readFourIRSensors(void)
{
    gCheckADC = false;
    DL_ADC12_startConversion(ADC12_0_INST);
    while (gCheckADC == false) { __WFE(); }

    sUp    = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    sRight = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    sDown  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    sLeft  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);

    DL_ADC12_disableConversions(ADC12_0_INST);  // <-- add this
    DL_ADC12_enableConversions(ADC12_0_INST);
}

// run scan
ScanResult runScan(void)
{
    ScanResult result = {
        .bestPanPos = PAN_POS_CENTER,
        .bestTiltPos = TILT_POS_CENTER,
        .bestSignal = 0,
        .horizontalError = 0,
        .verticalError = 0,
        .beaconFound = false
    };

    for (int32_t tilt = TILT_POS_MIN; tilt <= TILT_POS_MAX; tilt += TILT_STEP)
    {
        servoMoveTilt(tilt);
        delayServoSettle();

        for (int32_t pan = PAN_POS_MIN; pan <= PAN_POS_MAX; pan += PAN_STEP)
        {
            servoMovePan(pan);
            delayServoSettle();

            readFourIRSensors();


            /* totalSignal: just sum them up now, no inversion */
            uint32_t totalSignal = sUp + sRight + sDown + sLeft;

            /* Errors: now sRight HIGHER than sLeft means target on the right */
            int32_t horizontalError = (int32_t)sRight - (int32_t)sLeft;
            int32_t verticalError   = (int32_t)sUp    - (int32_t)sDown;

            if (totalSignal > result.bestSignal)
            {
                result.bestSignal = totalSignal;
                result.bestPanPos  = (uint16_t)pan;
                result.bestTiltPos = (uint16_t)tilt;
                result.horizontalError = horizontalError;
                result.verticalError   = verticalError;
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

        uint32_t total = sUp + sRight + sDown + sLeft;  // no inversion
        if (total < LOST_TOTAL_THRESHOLD) return false;

        int32_t panErr  = (int32_t)sRight - (int32_t)sLeft;   // un-flipped
        int32_t tiltErr = (int32_t)sUp    - (int32_t)sDown;   // un-flipped

        int32_t panU  = (int32_t)pidStep(&panPID,  (float)panErr,  dt);
        int32_t tiltU = (int32_t)pidStep(&tiltPID, (float)tiltErr, dt);

        panU  = clampStep(panU);
        tiltU = clampStep(tiltU);

        servoMovePanRaw(panPosRaw   + panU);
        servoMoveTiltRaw(tiltPosRaw + tiltU);

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
    return false;
}


int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
 
    const float dt = TRACK_TICK_MS / 1000.0f;
 
    printf("\r\n=== IR PID Bench Test ===\r\n");
    printf("Cols: U  D  L  R  | total | panErr tiltErr | panU tiltU | status\r\n");
    printf("--------------------------------------------------------------------\r\n");
 
    uint32_t tick = 0;
 
    while (1)
    {
        readFourIRSensors();
 
        /* Phototransistors: higher ADC = brighter IR (no inversion needed) */
        uint32_t total = (uint32_t)sUp + (uint32_t)sRight +
                         (uint32_t)sDown + (uint32_t)sLeft;
 
        int32_t panErr  = (int32_t)sRight - (int32_t)sLeft;
        int32_t tiltErr = (int32_t)sUp    - (int32_t)sDown;
 
        bool seeBeacon = (total >= LOST_TOTAL_THRESHOLD);
 
        float panU_f  = pidStep(&panPID,  (float)panErr,  dt);
        float tiltU_f = pidStep(&tiltPID, (float)tiltErr, dt);
 
        int32_t panU  = clampStep((int32_t)panU_f);
        int32_t tiltU = clampStep((int32_t)tiltU_f);
 
        const char *status;
        if (!seeBeacon)                       status = "NO SIGNAL";
        else if (panErr ==  0 && tiltErr == 0) status = "CENTERED";
        else if (panErr  >  0 && tiltErr >  0) status = "target UP-RIGHT";
        else if (panErr  >  0 && tiltErr <  0) status = "target DOWN-RIGHT";
        else if (panErr  <  0 && tiltErr >  0) status = "target UP-LEFT";
        else if (panErr  <  0 && tiltErr <  0) status = "target DOWN-LEFT";
        else if (panErr  >  0)                 status = "target RIGHT";
        else if (panErr  <  0)                 status = "target LEFT";
        else if (tiltErr >  0)                 status = "target UP";
        else                                   status = "target DOWN";
 
        printf("[%5lu] %4u %4u %4u %4u | %5lu | %+5ld %+5ld | %+4ld %+4ld | %s\r\n",
               (unsigned long)tick,
               sUp, sDown, sLeft, sRight,
               (unsigned long)total,
               (long)panErr, (long)tiltErr,
               (long)panU, (long)tiltU,
               status);
 
        tick++;
        delay_cycles(32000000UL * TRACK_TICK_MS / 1000UL);  // ~TRACK_TICK_MS at 32 MHz
    }
}

// int main(void)
// {
//     SYSCFG_DL_init();
//     NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

//     delay_cycles(3200000);   // ~100 ms boot delay for adapter

//     // start at center
//     servoMovePan(PAN_POS_CENTER);
//     servoMoveTilt(TILT_POS_CENTER);
//     delayServoSettle();

//     while (1)
//     {
//         ScanResult scan = runScan();

//         // if (!scan.beaconFound) continue;

//         // servoMovePan(scan.bestPanPos);
//         // servoMoveTilt(scan.bestTiltPos);
//         // delayServoSettle();

//         // if (fineTrack())
//         // {
//         //     while (fineTrack()) { /* keep tracking */ }
//         // }
//     }
// }

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
