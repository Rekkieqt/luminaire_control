#ifndef PID_H
#define PID_H

class pid{
    float h,b_h,b_l,c,K_h,K_l,Ti,Tt,Td,y_old,I,D;
    int N;
    float curr_b, curr_K, curr_r, G{0}, d{0};
    bool anti_wind_up{true}, feedback{true};
    float v{0};

  public:
    float r_h{10}, r_l{2};
  public:
    explicit pid(float h_, float b_h = 1, float b_l = 1, float c_ = 0, float K_h = 1, float K_l = 1, float Ti_ = 1, float Tt_ = 0.8, float Td_ = 1, int N_ = 10);
    ~pid(){};
    float get_reference(void);
    void set_reference(float r_);
    bool get_anti_wu_status(void);
    void set_anti_wu_status(bool st);
    bool get_fb_status(void);
    void set_fb_status(bool st);
    float get_disturbance(float u);
    void set_system_gain_n_dist(float G_, float d_);
    float computePWM(float y, float r);
    float computePWM(float y);
    void housekeep(float y);
};

inline void pid::housekeep(float y) {
  float e = curr_r - y;
  y_old = y;
  if(!feedback) return;
  I += curr_K*h/Ti*e;
}

#endif