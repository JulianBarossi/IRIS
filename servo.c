#include "servo.h"
#include "ti_msp_dl_config.h"

static void uart_send_byte(uint8_t byte)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST)) {}
    DL_UART_Main_transmitData(UART_1_INST, byte);
}

static void uart_send_packet(uint8_t *packet, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        uart_send_byte(packet[i]);
    }
}

uint8_t servo_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                       uint8_t *params, uint8_t param_count)
{
    uint16_t sum = id + length + instruction;

    for (uint8_t i = 0; i < param_count; i++) {
        sum += params[i];
    }

    return ~(sum & 0xFF);
}

void servo_ping(uint8_t id)
{
    uint8_t length = 0x02;
    uint8_t checksum = ~((id + length + SERVO_PING) & 0xFF);

    uint8_t packet[] = {
        0xFF, 0xFF,
        id,
        length,
        SERVO_PING,
        checksum
    };

    uart_send_packet(packet, sizeof(packet));
}

static bool uart_read_byte_timeout(uint8_t *byte, uint32_t timeout)
{
    while (timeout--) {
        if (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
            *byte = DL_UART_Main_receiveData(UART_1_INST);
            return true;
        }
    }

    return false;
}

bool servo_ping_read(uint8_t id)
{
    servo_ping(id);

    uint8_t resp[6];

    for (int i = 0; i < 6; i++) {
        if (!uart_read_byte_timeout(&resp[i], 3200000)) {
            return false;
        }
    }

    return resp[0] == 0xFF &&
           resp[1] == 0xFF &&
           resp[2] == id;
}

void servo_write_position(uint8_t id, uint16_t position,
                          uint16_t time, uint16_t speed)
{
    uint8_t params[] = {
        SERVO_ADDR_POS,
        (uint8_t)(position & 0xFF),
        (uint8_t)(position >> 8),
        (uint8_t)(time & 0xFF),
        (uint8_t)(time >> 8),
        (uint8_t)(speed & 0xFF),
        (uint8_t)(speed >> 8)
    };

    uint8_t length = sizeof(params) + 2;
    uint8_t checksum = servo_checksum(id, length, SERVO_WRITE,
                                      params, sizeof(params));

    uint8_t packet[13];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = length;
    packet[4] = SERVO_WRITE;

    for (uint8_t i = 0; i < sizeof(params); i++) {
        packet[5 + i] = params[i];
    }

    packet[12] = checksum;

    uart_send_packet(packet, sizeof(packet));
}