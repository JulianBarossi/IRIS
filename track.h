#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TRACKER_STATE_CENTER,
    TRACKER_STATE_SCAN,
    TRACKER_STATE_TRACK,
    TRACKER_STATE_LOST
} TrackerState;

extern volatile uint16_t sUp;
extern volatile uint16_t sRight;
extern volatile uint16_t sDown;
extern volatile uint16_t sLeft;

extern uint16_t g_irThreshold;
extern uint8_t g_panServoId;
extern uint8_t g_tiltServoId;

void tracker_init(void);
void tracker_update(void);
void tracker_adc_irq(void);

#endif