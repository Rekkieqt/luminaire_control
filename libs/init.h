#ifndef init_H
#define init_H
#include <cstdint>
#include "can.h"

enum static_parameters {
    BUFFER = 1000, //edit to fit 1 min of data
    R1 = 10000,
    Rled = 47,
    DAC_RES = 12,
    CAP1 = 10,
    WRITE_FREQ = 60000,
    LED_PIN = 15, // 
    DAC_RANGE = 4095, // write and read val
    R0 = 280000, // ldr at 10 lux
    Fs = 100,     //samplimg frequency
    MSG_SIZE = 8,  //
    Nfilter = 6, //measuring vout
    ONE_SEC_MS = 1000
};

enum inr_frm_header_types {
    //core0 to core1
    SEND_REG = 1,
    ERR_REQ = 2,
    ACK = 3,
    BROADCAST = 4,
    //core1 to core0
    ERR_INFO = 1,
    STAND_RECV = 2,
    RTR_FRAME = 3
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
    // init params
    RXpin = 6,
    TXpin = 5,
    CSpin = 7,
    SCKpin = 4,
    INTpin = 9,
    SPIclock = 10000000
};

union msg_to_can {
    uint8_t in_bytes[20];  // Raw byte access (20 bytes)
    uint32_t in_msg[5];    // Access as 32-bit words (5 * 4 = 20 bytes)

    struct msg_wrap {  // Nested struct
        uint8_t internal_msg[4];  // 4 bytes
        can_frame can_msg;        // 16 bytes
    } wrapped; // <-- Named instance inside the union
};

struct data_reads {
    volatile unsigned long time = 0.0;
    volatile float out = 0.0;
    volatile int ref = 0; 
    volatile float u = 0.0;
};

struct static_lux_data {
    float min_lux = 0.0;
    float max_lux = 0.0;
    float G = 0;
};

union msg_read {
    uint8_t input_ascii[MSG_SIZE];
    float input_value[MSG_SIZE];
};

enum CharToASCII {
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
    z_msg = 122   // Lowercase z
};

#endif //init_H