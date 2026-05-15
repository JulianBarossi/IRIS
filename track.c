#include "track.h"
#include "ti_msp_dl_config.h"
#include "servo.h"

volatile bool gCheckADC = false;

volatile uint16_t sUp = 0;
volatile uint16_t sRight = 0;
volatile uint16_t sDown = 0;
volatile uint16_t sLeft = 0;

uint16_t g_irThreshold = 120;

uint8_t g_panServoId = 0;
uint8_t g_tiltServoId = 1;

#define PAN_POS_MIN      1024
#define PAN_POS_MAX      3072
#define PAN_POS_CENTER   2048

#define TILT_POS_MIN     2100
#define TILT_POS_MAX     3000
#define TILT_POS_CENTER  2550

#define PAN_STEP         120
#define TILT_STEP        120

#define SERVO_SCAN_TIME   300
#define SERVO_TRACK_TIME  80
#define SERVO_SPEED       0

#define SERVO_SETTLE_CYCLES  5000000U
#define TRACK_DELAY_CYCLES   1000000U

#define LOST_THRESHOLD       80
#define ERROR_DEADBAND       25
#define TRACK_STEP_CLAMP     40

static TrackerState state = TRACKER_STATE_CENTER;

static int32_t panPos = PAN_POS_CENTER;
static int32_t tiltPos = TILT_POS_CENTER;

static int32_t clamp_int32(int32_t x, int32_t min, int32_t max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static void move_pan(int32_t pos, uint16_t time)
{
    panPos = clamp_int32(pos, PAN_POS_MIN, PAN_POS_MAX);
    servo_write_position(g_panServoId, panPos, time, SERVO_SPEED);
}

static void move_tilt(int32_t pos, uint16_t time)
{
    tiltPos = clamp_int32(pos, TILT_POS_MIN, TILT_POS_MAX);
    servo_write_position(g_tiltServoId, tiltPos, time, SERVO_SPEED);
}

static void read_ir_sensors(void)
{
    gCheckADC = false;

    DL_ADC12_startConversion(ADC12_0_INST);

    while (gCheckADC == false) {
        __WFE();
    }

    sUp    = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    sRight = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    sDown  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    sLeft  = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);

    DL_ADC12_disableConversions(ADC12_0_INST);
    DL_ADC12_enableConversions(ADC12_0_INST);
}

static uint32_t total_signal(void)
{
    return (uint32_t)sUp + sRight + sDown + sLeft;
}

static int32_t clamp_step(int32_t x)
{
    if (x > TRACK_STEP_CLAMP) return TRACK_STEP_CLAMP;
    if (x < -TRACK_STEP_CLAMP) return -TRACK_STEP_CLAMP;
    return x;
}

static bool scan_for_beacon(void)
{
    uint32_t bestSignal = 0;
    int32_t bestPan = PAN_POS_CENTER;
    int32_t bestTilt = TILT_POS_CENTER;

    for (int32_t tilt = TILT_POS_MIN; tilt <= TILT_POS_MAX; tilt += TILT_STEP) {
        move_tilt(tilt, SERVO_SCAN_TIME);
        delay_cycles(SERVO_SETTLE_CYCLES);

        for (int32_t pan = PAN_POS_MIN; pan <= PAN_POS_MAX; pan += PAN_STEP) {
            move_pan(pan, SERVO_SCAN_TIME);
            delay_cycles(SERVO_SETTLE_CYCLES);

            read_ir_sensors();

            uint32_t total = total_signal();

            if (total > bestSignal) {
                bestSignal = total;
                bestPan = pan;
                bestTilt = tilt;
            }
        }
    }

    if (bestSignal >= g_irThreshold) {
        move_pan(bestPan, SERVO_SCAN_TIME);
        move_tilt(bestTilt, SERVO_SCAN_TIME);
        delay_cycles(SERVO_SETTLE_CYCLES);
        return true;
    }

    return false;
}

static bool track_beacon_once(void)
{
    read_ir_sensors();

    uint32_t total = total_signal();

    if (total < LOST_THRESHOLD) {
        return false;
    }

    int32_t panErr = (int32_t)sRight - (int32_t)sLeft;
    int32_t tiltErr = (int32_t)sUp - (int32_t)sDown;

    if (panErr > ERROR_DEADBAND || panErr < -ERROR_DEADBAND) {
        int32_t panStep = clamp_step(panErr / 8);
        move_pan(panPos + panStep, SERVO_TRACK_TIME);
    }

    if (tiltErr > ERROR_DEADBAND || tiltErr < -ERROR_DEADBAND) {
        int32_t tiltStep = clamp_step(tiltErr / 8);
        move_tilt(tiltPos + tiltStep, SERVO_TRACK_TIME);
    }

    delay_cycles(TRACK_DELAY_CYCLES);
    return true;
}

void tracker_init(void)
{
    state = TRACKER_STATE_CENTER;

    panPos = PAN_POS_CENTER;
    tiltPos = TILT_POS_CENTER;

    move_pan(panPos, SERVO_SCAN_TIME);
    move_tilt(tiltPos, SERVO_SCAN_TIME);

    delay_cycles(SERVO_SETTLE_CYCLES);

    state = TRACKER_STATE_SCAN;
}

void tracker_update(void)
{
    switch (state) {
        case TRACKER_STATE_CENTER:
            move_pan(PAN_POS_CENTER, SERVO_SCAN_TIME);
            move_tilt(TILT_POS_CENTER, SERVO_SCAN_TIME);
            delay_cycles(SERVO_SETTLE_CYCLES);
            state = TRACKER_STATE_SCAN;
            break;

        case TRACKER_STATE_SCAN:
            if (scan_for_beacon()) {
                state = TRACKER_STATE_TRACK;
            } else {
                state = TRACKER_STATE_SCAN;
            }
            break;

        case TRACKER_STATE_TRACK:
            if (!track_beacon_once()) {
                state = TRACKER_STATE_LOST;
            }
            break;

        case TRACKER_STATE_LOST:
            move_pan(PAN_POS_CENTER, SERVO_SCAN_TIME);
            move_tilt(TILT_POS_CENTER, SERVO_SCAN_TIME);
            delay_cycles(SERVO_SETTLE_CYCLES);
            state = TRACKER_STATE_SCAN;
            break;

        default:
            state = TRACKER_STATE_CENTER;
            break;
    }
}

void tracker_adc_irq(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
            gCheckADC = true;
            break;

        default:
            break;
    }
}