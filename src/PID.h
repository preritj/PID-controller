#ifndef PID_H
#define PID_H

#include <valarray>

class PID {
private:
  bool isInitialized, isOptimize;
  std::valarray<double> error_; // PID errors
  std::valarray<double> K_; // PID coeeficients
  double dt; // time interval between frames
  double prev_cte; //CTE in previous frame
  
  //parameters for twiddle optimization :
  bool isBaseline; // true for the first run in twiddle algorithm 
  double tol, fpi; //tol:tolerance, fpi:frames per iteration
  std::valarray <double> dK_; //change in gain parameters for twiddle 
  int para_index; // index of parameter space to update
  double current_err; // current error in twiddle algorithm
  double best_err; // best (min) error in twiddle algorithm
  std::valarray<double> status;

public:
  int counter;
  double total_error;

  /*
  * Errors
  */
  std::valarray<double> error(); 

  /*
  * Coefficients
  */ 
  std::valarray<double> K();
  /*
  * Constructor
  */
  PID(std::valarray<double> para, double delta_t);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void reset();

  void optimize(double tolerance, int fpi, std::valarray<double> K_step);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void twiddle();
  void scan();
};

#endif /* PID_H */
