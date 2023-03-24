// NRF.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <ftd2xx.h>

#define NRF_CONFIG_PRIM_RX 0x1
#define NRF_CONFIG_PWR_UP 0x2
#define NRF_CONFIG_CRCO 0x4
#define NRF_CONFIG_EN_CRC 0x8
#define NRF_CONFIG_MASK_MAX_RT 0x10
#define NRF_CONFIG_MASK_TX_DS 0x20
#define NRF_CONFIG_MASK_RX_DR 0x40
#define NRF_CONFIG_RESERVED 0x80

#define NRF_STATUS_TX_FULL 0x1
#define NRF_STATUS_RX_P_NO 0xE // bits 1-3
#define NRF_STATUS_MAX_RT 0x10
#define NRF_STATUS_TX_DS 0x20
#define NRF_STATUS_RX_DR 0x40

#define NRF_REG_CONFIG 0x00
#define NRF_REG_EN_AA 0x01 // enable auto ack
#define NRF_REG_EN_RXADDR 0x02
#define NRF_REG_SETUP_AW 0x03
#define NRF_REG_SETUP_RETR 0x04
#define NRF_REG_RF_CH 0x05
#define NRF_REG_RF_SETUP 0x06
#define NRF_REG_STATUS 0x07
#define NRF_REG_OBSERVE_TX 0x08 
#define NRF_REG_CD 0x09
#define NRF_REG_FIFO_STATUS 0x17

#define NRF_CMD_WRITE 0x20
#define NRF_CMD_WRITE_PAYLOAD 0xA0
#define NRF_CMD_REUSE_TX 0xE0
#define NRF_CMD_FLUSH_TX 0xE0
#define NRF_CMD_FLUSH_RX 0xE0

#define NRF_REG_TX_ADDR 0x10 // 4 bytes

#define BIT_MOSI 0x1
#define BIT_SCK 0x2
#define BIT_CSN 0x4
#define BIT_CE 0x8
#define BIT_MISO 0x16

#define DATA_DEFAULT BIT_CSN

FT_HANDLE handle;
uint8_t curData;

void set_bit(uint8_t bit, uint8_t value) {
    if (value) {
        curData |= bit;
    }
    else {
        curData &= (~bit);
    }
}

void write_data(uint8_t data) {
    int writtenCount;
    FT_Write(handle, &data, 1, (LPDWORD)&writtenCount);
    curData = data;
}

void nrf_write_reg_multi(uint8_t reg, uint8_t cmd2, uint8_t *buf, uint8_t count) {
    set_bit(BIT_CSN, 0);
    set_bit(BIT_SCK, 0);
    write_data(curData);
    Sleep(1);

    uint8_t cmd = (reg & 0x1F) | cmd2;

    // write cmd
    for (int i = 0; i < 8; i++) {
        // load bit
        set_bit(BIT_SCK, 0);
        set_bit(BIT_MOSI, (cmd & (1 << (7 - i))) != 0);
        write_data(curData);
        Sleep(1);
        // transfer bit
        set_bit(BIT_SCK, 1);
        write_data(curData);
        Sleep(1);
    }
    set_bit(BIT_MOSI, 0);

    for (int j = 0; j < count; j++) {
        // write value
        for (int i = 0; i < 8; i++) {
            // load bit
            set_bit(BIT_SCK, 0);
            set_bit(BIT_MOSI, (buf[j] & (1 << (7 - i))) != 0);
            write_data(curData);
            Sleep(1);
            // transfer bit
            set_bit(BIT_SCK, 1);
            write_data(curData);
            Sleep(1);
        }
    }

    set_bit(BIT_SCK, 0);
    write_data(curData);
    Sleep(1);
    set_bit(BIT_CSN, 1);
    write_data(curData);
    Sleep(1);
}

void nrf_write_reg(uint8_t reg, uint8_t cmd, uint8_t value) {
    nrf_write_reg_multi(reg, cmd, &value, 1);
}

void nrf_read_reg_multi(uint8_t reg, uint8_t *buf, uint8_t count) {
    set_bit(BIT_CSN, 0);
    set_bit(BIT_SCK, 0);
    write_data(curData);
    Sleep(1);

    reg &= 0x1F;

    // send cmd
    for (int i = 0; i < 8; i++) {
        // load bit
        set_bit(BIT_SCK, 0);
        set_bit(BIT_MOSI, (reg & (1 << (7 - i))) != 0);
        write_data(curData);
        Sleep(1);
        // transfer bit
        set_bit(BIT_SCK, 1);
        write_data(curData);
        Sleep(1);
    }
    set_bit(BIT_MOSI, 0);

    for (int j = 0; j < count; j++) {
        uint8_t result = 0;
        for (int i = 0; i < 8; i++) {
            int readCount = 0;
            uint8_t current;
            // SCK low
            set_bit(BIT_SCK, 0);
            write_data(curData);
            Sleep(1);
            set_bit(BIT_SCK, 1);
            write_data(curData);
            Sleep(1);
            FT_GetBitMode(handle, &current);
            result |= ((current & BIT_MISO) >> 4) << (7 - i);
        }
        buf[j] = result;
    }
    set_bit(BIT_SCK, 0);
    write_data(curData);
    Sleep(1);
    set_bit(BIT_CSN, 1);
    write_data(curData);
    Sleep(1);
}

uint8_t nrf_read_reg(uint8_t reg) {
    uint8_t result;
    nrf_read_reg_multi(reg, &result, 1);
    return result;
}

void nrf_transmit() {
    set_bit(BIT_CE, 1);
    write_data(curData);
    Sleep(1);
    set_bit(BIT_CE, 0);
    write_data(curData);
    Sleep(1);
}

int main()
{
    auto result = FT_Open(0, &handle);
    if (FT_SUCCESS(result)) {
        std::cout << "Open success!\n";
        // mask 1 - output, 0 - input
        // 0 - reset, 1 async bitbang, 2 MPSSE, 4 sync bit bang, 8 - MCU host bus emulation, 16 - fast serial for opto isolation
        // NOTE: the more common 232RL doesn't support MPSSE
        FT_SetBitMode(handle, 0b11101111, 1);
        FT_SetBaudRate(handle, 9600);
        // bits - RIN, DCD, DSR, DTR, CTS, RTS, RXD, TXD
        // on nrf
        //                       MISO  CE  CSN SCK MOSI

        uint8_t data[255];
        data[0] = 0x03;
        data[1] = 0x00;
        uint32_t writtenCount = 0;

        write_data(DATA_DEFAULT);

        /*while (true) {
            uint8_t value;
            FT_GetBitMode(handle, &value);
            printf("Read %d bytes, value 0x%x\n", writtenCount, value);
            Sleep(100);
        }*/

        uint8_t txAddr[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
        uint8_t txAddrAck[4];
        uint8_t payload[] = { 0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
            0xFA, 0xCE, 0xFE, 0xED,
        };

        while (true) {
            printf("startup sequence...\n");
            uint8_t status = nrf_read_reg(NRF_REG_STATUS);
            Sleep(3);
            printf("Status is %x\n", status);
            uint8_t rfCh = nrf_read_reg(NRF_REG_RF_CH);
            printf("RF freq: %d Mhz\n", 2400 + rfCh);
            nrf_write_reg(NRF_REG_RF_SETUP, NRF_CMD_WRITE, 0x07); // 1mW, 1Mbps
            uint8_t rfSetup = nrf_read_reg(NRF_REG_RF_SETUP);
            printf("RF setup: %x\n", rfSetup);
            // power down -> standby takes max 1.5ms
            nrf_write_reg(NRF_REG_CONFIG, NRF_CMD_WRITE, NRF_CONFIG_PWR_UP);
            Sleep(2);
            uint8_t config = nrf_read_reg(NRF_REG_CONFIG);
            printf("Config after power up: %x\n", config);
            Sleep(2);
            // disable auto ack
            nrf_write_reg(NRF_REG_EN_AA, NRF_CMD_WRITE, 0x00);
            // disable automatic retransmission
            nrf_write_reg(NRF_REG_SETUP_RETR, NRF_CMD_WRITE, 0x00);
            uint8_t retr = nrf_read_reg(NRF_REG_SETUP_RETR);
            printf("setup retr: %x\n", retr);
            Sleep(1);
            nrf_write_reg_multi(NRF_REG_TX_ADDR, NRF_CMD_WRITE, txAddr, 4);
            Sleep(1);
            nrf_read_reg_multi(NRF_REG_TX_ADDR, txAddrAck, 4);
            printf("Addr is %x%x%x%x\n", txAddrAck[0], txAddrAck[1], txAddrAck[2], txAddrAck[3]);
            Sleep(1);

            /*for (int i = 0; i < 3; i++) */{
                nrf_write_reg_multi(0x0, NRF_CMD_WRITE_PAYLOAD, payload, 4);
                //Sleep(1);
                status = nrf_read_reg(NRF_REG_STATUS);
                printf("Status before transmit: %x\n", status);
                /*nrf_transmit();
                do {
                    status = nrf_read_reg(NRF_REG_STATUS);
                    printf("Status after transmit: %x\n", status);
                    if ((status & 0x20) != 0)
                        break;
                } while (true);
                // clear status bits
                nrf_write_reg(NRF_REG_STATUS, NRF_CMD_WRITE, status);*/

                // burst last packet for 100ms
                nrf_write_reg_multi(0x3, 0xE0, NULL, 0);
                uint8_t fifoStatus = nrf_read_reg(NRF_REG_FIFO_STATUS);
                printf("fifo status: %x\n", fifoStatus);
                set_bit(BIT_CE, 1);
                write_data(curData);
                Sleep(1);
                Sleep(10000);
                set_bit(BIT_CE, 0);
                write_data(curData);
                Sleep(1);
                nrf_write_reg_multi(0x1, NRF_CMD_FLUSH_TX, NULL, 0);
                fifoStatus = nrf_read_reg(NRF_REG_FIFO_STATUS);
                printf("fifo status after: %x\n", fifoStatus);
            }
            uint8_t obs = nrf_read_reg(0x08);
            printf("obs: %x\n", obs);
            Sleep(500);
        }
    }
    else std::cout << "Open fail!\n";
    std::cout << "Exiting...\n";
    return 0;
}