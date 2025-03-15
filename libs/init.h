#ifndef init_H
#define init_H

enum static_parameters {
    BUFFER = 100, //edit to fit 1 min of data
    DAC_RES = 12,
    WRITE_FREQ = 60000,
    LED_PIN = 15,
    DAC_RANGE = 4095,
    R1 = 225000,
    Fs = 10000,     //samplimg frequency
    Freq = 100,     //sin ref signal frequency
    MSG_SIZE = 3,  //
    Nfilter = 5
};

struct data_reads {
    float time;
    float out;
    int ref;
    float u;
};

union msg_read {
    int input_ascii[MSG_SIZE];
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