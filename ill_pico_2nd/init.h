#ifndef init_H
#define init_H
#include <cstdint>
#include "can.h"
#include "init.h"
#include "pid.h"

/*---------- common.h ----------*/
#define LDR_PIN A0

#define OCC true
#define UNOCC false

#define VCC 3.3 // V
#define _R 10e3 // Ohm

#define SAMPLE_TIME 10 // ms

#define CIRC_NUM 1

#define SERIAL_PRINTS_0 0 
#define SERIAL_PRINTS_1 0
/*-----------------------------*/

/*---------- state.h ----------*/
extern pid PID;
extern bool occ_st;

extern unsigned long last_restart;
/*-----------------------------*/

/*---------- flags.h ----------*/
extern bool print_y;
extern bool print_u;
extern bool print_buff;

extern char x; // character representing variable to print form buffer
/*-----------------------------*/

enum static_parameters {
    BUFFER = 6000, //edit to fit 1 min of data
    R1 = 10000, // 10k ohm resistance of ldr circuit
    Rled = 47, // ohm the resistance of led circuit
    DAC_RES = 12, //12 bit dac resolution
    CAP1 = 10, //10 micro farads
    WRITE_FREQ = 60000, //pwm write frequency 
    LED_PIN = 15, // controlled led
    DAC_RANGE = 4096, // write and read val
    R0_ldr = 275000, // ldr at 10 lux
    Fs = 100,     //samplimg frequency
    MSG_SIZE = 9,  // Serial bytes read on commands
    Nfilter = 11, //measuring vout
    ONE_SEC_MS = 1000 // one second wait in millis
};

enum inr_frm_header_types { // type of messages between cores through internal fifo
    //core0 to core1
    REQUEST = 0xff,
    ERR_REQ = 0x0f,
    ACK = 0x00,
    HEAD_FLAG = 0x13,
    CRITICAL_ERRORS = 0xaa,
    ERRORS = 0xbb
    //global headers
};

enum canbus_parameters {
    // init used pins and spi clock def
    SCKpin = 2,
    TXpin = 3,
    RXpin = 4,
    CSpin = 5,
    INTpin = 6,
    SPIclock = 10000000
};

union msg_to_can { //internal frame between core comunications
    uint8_t in_bytes[20];  
    uint32_t in_msg[5];   

    struct msg_wrap {  // Nested struct
        uint8_t internal_msg[4];  // 4 bytes for irq errors and msge between cores
        can_frame can_msg;        // 16 bytes
    } wrapped; // Named instance inside the union
};

struct data_reads { // data written by the controller loop seq
    volatile uint32_t time{0};
    volatile float out{0};
    volatile int ref{0}; 
    volatile float u{0};
    volatile float sim_out{0};
};

#endif //init_H