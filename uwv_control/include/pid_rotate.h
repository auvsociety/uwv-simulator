#ifndef PID_ROTATE_H
#define PID_ROTATE_H

#include <cmath>

/**
 * @brief PID controller class for rotational motion of the underwater vehicle.
 */

class PidRotate {
private:
  /// @brief Previous PID error
  double prev_err_;

  /// @brief PID output value
  double output_;

  /// @brief P, I, D error terms
  double p_, i_, d_;
  
  /**
   * @brief Rounds a number to the number of decimals points required.
   * 
   * @param _f Value to be rounded
   * @param _decimals Number of decimals places
   * 
   * @return Rounded number
   */
  double f_round(double _f, int _decimals);

public:
  /// @brief Constructor
  PidRotate();

  /// @brief PID constants
  double kp_;   ///< Proportional (Kp)
  double ki_;   ///< Integral (Ki)
  double kd_;   ///< Derivative (Kd)

  /**
   * @brief Used to define the range around the set-point until
   *        which if the current state exists, the loop considers
   *        that it has reached the set point.
   */ 
  double snstvty_;

  /// @brief Upper and lower integral windup limit; must be positive real number
  double int_windup_limit_;

  /// @brief Upper and lower PID output limits; must be positive real number
  double output_limit_;


  /**
   * @brief Runs the PID loop once
   * 
   * @param _set_point Set point
   * @param _cur_state Current state
   * @param _dt Time difference between previous run and current run
   * 
   * @return PID output value
   */ 
  double update(double _set_point, double _cur_state, double _dt);

  /**
   * @brief Resets the PID error terms to zero
   */ 
  void reset();
};

#endif
