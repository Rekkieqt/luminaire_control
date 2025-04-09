#ifndef init_H
#define init_H
#include <cstdint>
#include "can.h"
#include "pid.h"
#include "optm.h"

extern uint8_t myIdentifier;
extern uint8_t maxId;

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
extern optimizer nice;
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
    FIVE_SEC = 5000, // five second wait in millis
    THIRTY_SEC = 30000 // thirty second wait in millis
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
    OPTIMIZATION = 0x03, //for consensus control
    STREAM = 0X05,
    // 3 remaining types 
/*-----------------------------*/

/*---------- GENERIC HEADER FLAGS ----------*/
    ACK = 0x00, //msg for acceptance
    REQ = 0x01, //msg for request
    ERR = 0x02, //msg for error
    UNDEFINED = 0x03, // for an additional message
/*-----------------------------*/
/*---------- STREAM SPECIFIC HEADER FLAGS ----------*/
    LUX = 0x02, // for y 
    MIU = 0x03, // for u
/*-----------------------------*/
/*---------- OPTIMIZATION SPECIFIC HEADER FLAGS ----------*/
    LAMBDA = 0x02, // for lmbd 
    INPUT_U = 0x03, // for u
/*-----------------------------*/
/*---------- CALIBRATION SPECIFIC HEADER FLAGS ----------*/
    START = 0x01, // for y 
    END = 0x02, // for u
    UNDEFC = 0x03, // for u
/*-----------------------------*/
/*---------- SERIAL SPECIFIC HEADER FLAGS ----------*/
    GETS = 0x01, // for y 
    SETS = 0x02, // for u
    UNDEFS = 0x03, // for u
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
    SPIclock = 10000000,
    ID_MASK = 0x38
};

union __attribute__((aligned(4))) msg_to_can { //internal frame between core comunications
    uint8_t in_bytes[20];  
    uint32_t in_msg[5];   

    struct __attribute__((packed)) msg_wrap {  // Nested struct
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

struct ser_data { // data written by the controller loop seq
    float N{0};
    float visibility{0};
    float energy{0}; 
    float flicker{0};
    float voltage{0};
};

struct node_data {
    uint8_t id;
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
    // Sets (0x01 - 0x05)
    set_ref = 0x01,
    set_u,
    set_occ,
    set_aa,
    set_fb,

    // Gets (0x06 - 0x11)
    get_ref,
    get_u,
    get_y,
    get_volt,
    get_occ,
    get_aa,
    get_fb,
    get_dist,
    get_pwr,
    get_Rtime,
    get_buff_u,
    get_buff_y,

    // Performance metrics (0x12 - 0x14)
    get_flicker,
    get_vis,
    get_energy,

    // Streams (0x15 - 0x18)
    start_stream_u,
    start_stream_y,
    stop_stream_u,
    stop_stream_y,

    // Distribution (0x19 - 0x1F)
    get_lower_bound_occ,
    set_lower_bound_occ,
    get_lower_bound_unocc,
    set_lower_bound_unocc,
    get_curr_cost,
    get_curr_lum,
    set_curr_cost,

    // Reset
    reset = 0xFF
};


#endif //init_H