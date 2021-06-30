#include "pid_rotate.h"

PidRotate::PidRotate()
{
  kp_ = 10.0;
  kd_ = 5.0;
  ki_ = 0.01;
  snstvty_ = 0.002;
}

double PidRotate::update(double _set_point, double _cur_state, double _dt)
{
  _cur_state = f_round(_cur_state, 3);

  // update proportional, differential and integral errors
  p_ = _set_point - _cur_state;  // current error
  double abs_p = fabs(p_);
  if (fabs(p_) <= snstvty_)
    p_ = 0.0;
  else if(abs_p > 180) p_ = copysign(360 - abs_p, -p_);
  
  i_ = i_ + _dt * p_;          // i -> sum of prev errors
  d_ = (p_ - prev_err_) / _dt;  // d -> rate of error

  prev_err_ = p_;

  // update control output
  output_ = kp_ * p_ + kd_ * d_ + ki_ * i_;

  return output_;
}

void PidRotate::reset()
{
  p_ = i_ = d_ = output_ = prev_err_ = 0;
}

double PidRotate::f_round(double f, int decimals)
{
  float value = (int)(f * pow(10, decimals) + .5);
  return (float)value / pow(10, decimals);
}