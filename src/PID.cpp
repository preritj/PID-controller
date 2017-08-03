#include "PID.h"
#include "iostream"
#include "math.h"
#include <valarray>
#include <numeric>
#include <stdio.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(valarray<double> K, double delta_t)
{ 
  K_ = K;
  reset();
  dt = delta_t;
  isOptimize = false;
  status = std::valarray<double> (3); //sets status to 0
}

PID::~PID() {}

void PID::reset() {
  error_ = std::valarray<double> (3); //sets PID-errors to 0
  isInitialized = false;
  counter = 0;
  total_error = 0.;
}

void PID::optimize(double tolerance, 
                   int frames_per_iter, 
                   valarray<double> K_step){
  isOptimize = true;
  para_index = 0;
  tol = tolerance;
  fpi = frames_per_iter;
  isBaseline = true;
  dK_ = K_step;
}

void PID::UpdateError(double cte) {
  error_[0] = cte; // P-error
  error_[1] += cte*dt; // I-error
  // D-error
  if (!isInitialized){
    error_[2] = 0.; 
    isInitialized = true;
  }
  else{
    error_[2] = (cte - prev_cte)/dt;
  }  
  prev_cte = cte;

  if (isOptimize){
    total_error += pow(cte, 2.);
    counter++;
    if (counter == fpi)
      twiddle();
  }
}
 
void PID::twiddle(){
  current_err = total_error/fpi; 
  printf ("Current gain parameters %4.2f %4.2f %4.2f \n", 
                K_[0], K_[1], K_[2]);
  printf ("Current error %6.2f \n", current_err*100.); 
  // set baseline error
  if (isBaseline){
    best_err = current_err;
    isBaseline = false;
  }
  // coordinate descent 
  else {
    double sum_dK = dK_.sum();
    // check if tolerance met
    if (sum_dK < tol){
      printf ("*********************  \n");
      printf ("Optimization complete. \n");
      isOptimize = false;
      return;
    }
  }
  scan();
  reset();
}

void PID::scan(){
  if (status[para_index] == 0){
    K_[para_index] += dK_[para_index];
    status[para_index] = 1;
    return;
  }
  else if (status[para_index] == 1){
    // if smaller error :
    if (current_err < best_err){
      best_err = current_err;
      dK_[para_index] *= 1.1 ;
      status[para_index] = 0;
      para_index = (para_index+1) % 3;
      scan();
      return;
    }
    // if larger error :
    else{
      K_[para_index] -= 2.*dK_[para_index];
      status[para_index] = 2;
      return;
    }
  }
  else {
    if (current_err < best_err){
      best_err = current_err;
      dK_[para_index] *= 1.1;
    }
    else {
      K_[para_index] += dK_[para_index];
      dK_[para_index] *= 0.9;
    }
    status[para_index] = 0;  
    para_index = (para_index+1) % 3;
    scan();
    return;
  }
}

valarray<double> PID::K(){
  return K_;
}

valarray<double> PID::error(){
  return error_;
}

