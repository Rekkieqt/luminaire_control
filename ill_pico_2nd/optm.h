#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <cmath>

int total_nodes{3};
float Q{1e-2};
float my_cost{1};
int my_id{2};

// algorithm : calc new u solution -> broadcast -> keep on -> if(response arrives) increment response counter + update values -> 
// when(response counter = total_nodes - 1) calc new lbd solution -> if(not converged) broadcast; else *stop* |(not converged)-> keep on ->
// if(response arrives) increment response counter + update values -> when(response counter = total_nodes -1) repeat

void constraint(float& h, const float* x, const float*A, const float b, const int my_id){
  float line_sum{0};
  for(int j = 0; j < total_nodes; j++){
    line_sum = A[my_id * total_nodes + j]*x[j]; // A = -K
  }
  h = line_sum + b; // b = L-d
}

class optimizer{
  private:
    int n_consts{0};
    float *lbd{nullptr}, *u{nullptr}, cost, Q{1e-2}, *A{nullptr}, b{0};
    float h{0}, prev_h{0}, ascent_gain{1e-3};

    float threshold{1e-5};

    void (*constraint_fnc)(float&, const float*, const float*, const float , const int){nullptr};
  public:
    explicit optimizer(float cost_ = {1}, float Q_ = {1}, int n_consts_ = {0}, void (*constraint_fnc_)(float&, const float*, const float*, const float, const int) = nullptr);
    ~optimizer(){
      delete[] A;
      delete[] lbd;
      delete[] u;
    };

    void set_cost(float c);
    void set_threshold(float t);
    void set_constraints(float* A_, float b_);
    void update_lbd(float new_lbd, int id);
    void update_u(float new_u, int id);
    void iterate_primal(void);
    bool iterate_dual(void);
    float constraint_norm(const float h, const int n_consts);
    void new_ascent_gain(const float h, const float h_prev);
};


#endif