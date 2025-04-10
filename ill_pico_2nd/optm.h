#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <cmath>
#include <cstdint>

extern uint8_t maxId;
extern uint8_t myIdentifier;

// algorithm : calc new u solution -> broadcast -> keep on -> if(response arrives) increment response counter + update values -> 
// when(response counter = total_nodes - 1) calc new lbd solution -> if(not converged) broadcast; else *stop* |(not converged)-> keep on ->
// if(response arrives) increment response counter + update values -> when(response counter = total_nodes -1) repeat

void constraint(float& h, const float* x, const float*A, const float b, const int my_id);

class optimizer{
  private:
    int n_consts{0};
    float *lbd{nullptr}, *u{nullptr}, cost, Q{1}, *A{nullptr}, b{0};
    float h{0}, prev_h{0}, ascent_gain{4e-3};

    float threshold{1e-5};

    void (*constraint_fnc)(float&, const float*, const float*, const float , const int){nullptr};
  public:
    int n_replies{0};

    explicit optimizer(float cost_ = {1}, float Q_ = {1}, int n_consts_ = {0}, void (*constraint_fnc_)(float&, const float*, const float*, const float, const int) = constraint);
    ~optimizer(){
      delete[] A;
      delete[] lbd;
      delete[] u;
    };

    void set_cost(float c);
    void set_threshold(float t);
    void set_cnstr_fn(int n_consts_ , void (*constraint_fnc_)(float&, const float*, const float*, const float, const int));
    void set_cnstr_fn(int n_consts_);
    void set_constraints(float* A_, float b_);
    float get_sol(void);
    void update_lbd(float new_lbd, uint8_t id);
    void update_u(float new_u, uint8_t id);
    float iterate_primal(void);
    bool iterate_dual(float& lbd_);
    float constraint_norm(const float h, const int n_consts);
    void new_ascent_gain(const float h, const float h_prev);
};


#endif