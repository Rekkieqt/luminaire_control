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

extern bool SERIAL_PRINTS_0;
extern bool SERIAL_PRINTS_1;
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
extern bool dn_special_print;

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
    Fs = 100,     //samplimg frequency
    MSG_SIZE = 9,  // Serial bytes read on commands
    Nfilter = 11, //measuring vout
    ONE_SEC_MS = 1000, // one second wait in millis
    TEN_SEC = 10000, // one second wait in millis
    FIVE_SEC = 5000 // one second wait in millis
};

enum inr_frm_header_types { // type of messages between cores through internal fifo
    //core0 to core1
    REQUEST = 0xff,
    CAN_REG = 0x0f,
    HEAD_FLAG = 0x13,
    CRITICAL_ERRORS = 0xaa,
    ERRORS = 0xbb
};
enum can_bus_headers { // type of can message
/*---------- HEADERS ----------*/
    BOOT = 0x00, //generic
    CALIBRATION = 0x01, //for calibration
    SER_COM = 0x02, //for serial
    REFERENCE = 0x03, //for consensus control
    RESTART = 0x04, //for consensus control
    // 4 remaining types 
/*-----------------------------*/

/*---------- HEADER FLAGS ----------*/
    ACK = 0x00, //msg for acceptance
    REQ = 0x01, //msg for request
    ERR = 0x02, //msg for error
    UNDEFINED = 0x03, // for an additional message
/*-----------------------------*/
/*---------- SPECIAL ADRESSES ----------*/
    BROADCAST = 0x00 
/*-----------------------------*/
};

enum canbus_parameters {
    // init used pins and spi clock def
    SCKpin = 18,
    TXpin = 19,
    RXpin = 16,
    CSpin = 17,
    INTpin = 20,
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
    volatile float ref{0}; 
    volatile float u{0};
    volatile float sim_out{0};
};

struct node_data {
    int id;
    float G;
};

struct id_data
{
    uint8_t receiver{0};
    uint8_t sender{0};
    uint8_t header{0};
    uint8_t header_flag{0};
};

union can_data_decoder {   //can data decoding
    uint8_t bytes[8];
    bool bools[8];  
    uint32_t four_bytes[2];   
    uint64_t eight_bytes;   
    float floats[2];   
    int ints[2];   
};

enum serial_canbus_requests {
    // Sets (0x01 - 0x03)
    set_ref = 0x01,
    set_u = 0x02,
    set_occ = 0x03,
    set_aa = 0x22,
    set_fb = 0x23,

    // Gets (0x04 - 0x13)
    get_ref = 0x04,
    get_u = 0x05,
    get_y = 0x06,
    get_volt = 0x07,
    get_occ = 0x08,
    get_aa = 0x09,
    get_fb = 0x0A,
    get_dist = 0x0B,
    get_pwr = 0x0C,
    get_Rtime = 0x0D,
    get_buff_u = 0x0E,
    get_buff_y = 0x0F,

    // Performance metrics (0x14 - 0x16)
    get_flicker = 0x14,
    get_vis = 0x15,
    get_energy = 0x16,

    // Streams (0x17 - 0x1A)
    start_stream_u = 0x17,
    start_stream_y = 0x18,
    stop_stream_u = 0x19,
    stop_stream_y = 0x1A,

    // Distribution (0x1B - 0x21)
    get_lower_bound_occ = 0x1B,
    set_lower_bound_occ = 0x1C,
    get_lower_bound_unocc = 0x1D,
    set_lower_bound_unocc = 0x1E,
    get_curr_cost = 0x1F,
    get_curr_lum = 0x20,
    set_curr_cost = 0x21,

    // Reset (unchanged)
    reset = 0xFF
};


#endif //init_H