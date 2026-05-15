#include "ti_msp_dl_config.h"
#include "servo.h"
#include "track.h"
#include <stdio.h>


#define main1


#ifdef move
int main(void) {
    SYSCFG_DL_init();
    delay_cycles(3200000);

    while (1) {
        servo_write_position(1, 1800, 1000, 0);
        delay_cycles(64000000);
        servo_write_position(1, 2400, 1000, 0);
        delay_cycles(64000000);
    }
}
#endif


#ifdef ping


void send_packet(uint8_t *packet, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {

        while (DL_UART_isTXFIFOFull(UART_1_INST)) {}

        DL_UART_Main_transmitData(UART_1_INST, packet[i]);
    }
}

volatile uint8_t debug_byte = 0;

int main(void)
{
    SYSCFG_DL_init();

    delay_cycles(3200000);

    // Force servo ID to 1 using broadcast
uint8_t set_id_packet[] = {
    0xFF, 0xFF,
    0xFE,
    0x04,
    0x03,
    0x05,
    0x00,
    0xF5
};

    send_packet(set_id_packet, sizeof(set_id_packet));

    delay_cycles(3200000);

    // Ping ID 1
    uint8_t ping_packet[] = {
        0xFF, 0xFF,
        0x01,
        0x02,
        0x01,
        0xFB
    };

    send_packet(ping_packet, sizeof(ping_packet));

    while (1) {

        if (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {

            debug_byte = DL_UART_Main_receiveData(UART_1_INST);

            // BREAKPOINT HERE
            __asm("nop");
        }
    }
}
#endif


#ifdef main1

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    tracker_init();

    while (1) {
        tracker_update();
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    tracker_adc_irq();
}

#endif