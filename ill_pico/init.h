#ifndef init_H
#define init_H
#include <cstdint>
#include "can.h"

enum static_parameters {
    BUFFER = 6000, //edit to fit 1 min of data
    R1 = 10000, // 10k ohm resistance of ldr circuit
    Rled = 47, // ohm the resistance of led circuit
    DAC_RES = 12, //12 bit dac resolution
    CAP1 = 10, //10 micro farads
    WRITE_FREQ = 60000, //pwm write frequency 
    LED_PIN = 15, // controlled led
    DAC_RANGE = 4095, // write and read val
    R0 = 275000, // ldr at 10 lux
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
    //masks n filters
    mask0 = 1,
    mask1 = 1,
    rxf0 = 0,
    rxf1 = 0,
    rxf2 = 0,
    rxf3 = 0,
    rxf4 = 0,
    rxf5 = 0,
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

struct static_lux_data { // data that is created by luxmeter setup sequence
    float min_lux = 0.0;
    float max_lux = 0.0;
    float G = 0;
};

struct perf_meas { // data that is created by luxmeter setup sequence
    float flicker{0};
    float visibility{0};
    float energy{0};
    float Pmax{0};
};

struct user_set_flags { // data that is created by luxmeter setup sequence
    volatile bool pid{true};
    volatile bool feed_backward{true};
    volatile bool anti_windup{true};
    volatile bool occupancy{false}; 
    volatile bool rt_stream_out{false};
    volatile bool rt_stream_u{false};
};

union msg_read { //serial read buffer
    uint8_t numb[MSG_SIZE];
    char letter[MSG_SIZE];
};

enum CharToASCII { //
    A_msg = 65,   // Uppercase A
    B_msg = 66,   // Uppercase B
    C_msg = 67,   // Uppercase C
    D_msg = 68,   // Uppercase D
    E_msg = 69,   // Uppercase E
    F_msg = 70,   // Uppercase F
    G_msg = 71,   // Uppercase G
    H_msg = 72,   // Uppercase H
    I_msg = 73,   // Uppercase I
    J_msg = 74,   // Uppercase J
    K_msg = 75,   // Uppercase K
    L_msg = 76,   // Uppercase L
    M_msg = 77,   // Uppercase M
    N_msg = 78,   // Uppercase N
    O_msg = 79,   // Uppercase O
    P_msg = 80,   // Uppercase P
    Q_msg = 81,   // Uppercase Q
    R_msg = 82,   // Uppercase R
    data_stream_stop_msg = 83,   // Uppercase S
    T_msg = 84,   // Uppercase T
    U_msg = 85,   // Uppercase U
    V_msg = 86,   // Uppercase V
    W_msg = 87,   // Uppercase W
    X_msg = 88,   // Uppercase X
    Y_msg = 89,   // Uppercase Y
    Z_msg = 90,   // Uppercase Z
    
    a_msg = 97,   // Lowercase a
    b_msg = 98,   // Lowercase b
    c_msg = 99,   // Lowercase c
    dist_msg = 100,  // Lowercase d
    e_msg = 101,  // Lowercase e
    f_msg = 102,  // Lowercase f
    get_msg = 103,  // Lowercase g
    h_msg = 104,  // Lowercase h
    i_msg = 105,  // Lowercase i
    j_msg = 106,  // Lowercase j
    k_msg = 107,  // Lowercase k
    l_msg = 108,  // Lowercase l
    m_msg = 109,  // Lowercase m
    n_msg = 110,  // Lowercase n
    occupancy_msg = 111,  // Lowercase o
    p_msg = 112,  // Lowercase p
    q_msg = 113,  // Lowercase q
    ref_msg = 114,  // Lowercase r
    data_stream_start_msg = 115,  // Lowercase s
    time_msg = 116,  // Lowercase t
    duty_msg = 117,  // Lowercase u
    volt_msg = 118,  // Lowercase v
    w_msg = 119,  // Lowercase w
    x_msg = 120,  // Lowercase x
    lux_msg = 121,  // Lowercase y
    z_msg = 122,   // Lowercase z
    // numbers
    _0_msg = 48,
    _1_msg = 49,
    _2_msg = 50,
    _3_msg = 51,
    _4_msg = 52,
    _5_msg = 53,
    _6_msg = 54,
    _7_msg = 55,
    _8_msg = 56,
    _9_msg = 57
};

#endif //init_H