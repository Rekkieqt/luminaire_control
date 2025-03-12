#ifndef communicator_H
#define communicator_H
#define COMMAND_SIZE 3

enum CharToASCII {
    U_msg = 85,   // Uppercase U
    G_msg = 71,   // Uppercase G
    R_msg = 82,   // Uppercase R
    Y_msg = 89,   // Uppercase Y
    V_msg = 86,   // Uppercase V
    O_msg = 79,   // Uppercase O
    A_msg = 65,   // Uppercase A
    F_msg = 70,   // Uppercase F
    P_msg = 80,   // Uppercase P
    T_msg = 84,   // Uppercase T
    S_msg = 83,   // Uppercase S
    L_msg = 76,   // Uppercase L
    C_msg = 67,   // Uppercase C

    u_msg = 117,  // Lowercase u
    g_msg = 103,  // Lowercase g
    r_msg = 114,  // Lowercase r
    y_msg = 121,  // Lowercase y
    v_msg = 118,  // Lowercase v
    o_msg = 111,  // Lowercase o
    a_msg = 97,   // Lowercase a
    f_msg = 102,  // Lowercase f
    p_msg = 112,  // Lowercase p
    t_msg = 116,  // Lowercase t
    s_msg = 115,  // Lowercase s
    l_msg = 108,  // Lowercase l
    c_msg = 99    // Lowercase c
};
class communicator 
{
//inline defs
  private:
    int i;
    int command[COMMAND_SIZE];
    float val; 
    double time;
  public:
    //constructor
    communicator();
    //destructor
    ~communicator();

    //function
    void decoder_();
    double get_();
    double set_();
    
};

//function calls 
/*
  double get_offset(int a, int b);
  void tf_sweep();
  void calibrate();
    
*/
#endif //communicator_H
