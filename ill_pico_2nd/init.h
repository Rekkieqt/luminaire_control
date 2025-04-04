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
    TEN_SEC = 10000 // one second wait in millis
};

enum inr_frm_header_types { // type of messages between cores through internal fifo
    //core0 to core1
    REQUEST = 0xff,
    ERR_REQ = 0x0f,
    HEAD_FLAG = 0x13,
    CRITICAL_ERRORS = 0xaa,
    ERRORS = 0xbb,
    //global headers
    BROADCAST = 0x00
};
enum can_bus_frame_headers { // type of can message
/*---------- HEADERS ----------*/
    BOOT = 0x00, //generic
    CALIBRATION = 0x01, //for calibration
    SER = 0x02, //for serial
    REF = 0x03, //for consensus control
    // 4 remaining types 
/*-----------------------------*/

/*---------- HEADER FLAGS ----------*/
    ACK = 0x00, //msg for acceptance
    REQ = 0x01, //msg for request
    ERR = 0x02, //msg for error
    UNDEFINED = 0x03 // for an additional message
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
    uint8_t receiver;
    uint8_t sender;
    uint8_t header;
    uint8_t header_flag;
};

enum serial_canbus_requests {
    // sets
    set_ref = 0x01,
    set_u = 0x02,
    // gets
    get_ref = 0x03,
    get_u = 0x04,
    get_y = 0x05,
    get_volt = 0x06,
    get_occ = 0x07,
    get_aa = 0x08,
    get_fb = 0x09,
    get_dist = 0x0a,
    get_pwr = 0x0b,
    get_Rtime = 0x0c,
    get_buff = 0x0d,
    // perf
    get_flicker = 0x14,
    get_vis = 0x15,
    get_enrgy = 0x16,
    // streams
    start_stream = 0x17,
    stop_stream = 0x18,
    // distr
    get_lower_bound_occ = 0x19,
    set_lower_bound_occ = 0x1a,
    get_lower_bound_unocc = 0x1b, 
    set_lower_bound_unocc = 0x1c,
    get_curr_lum = 0x1e,
    get_curr_cost = 0x1d,
    set_curr_cost = 0x1f,
    reset = 0xff
      
};

#endif //init_H