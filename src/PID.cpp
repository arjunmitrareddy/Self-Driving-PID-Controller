#include <iostream>
#include <stdlib.h>
#include "PID.h"

#define DEQUE_LIMIT 20


using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  //for maintaining moving average
  deque_elems = (int*) calloc(DEQUE_LIMIT, sizeof(deque_elems[0]));
  i = 19;
  sum = 0;
}

void PID::UpdateError(double cte, double dt) {
  d_error = (cte - p_error) / dt;
  p_error = cte;
  i_error = add_err(cte * dt);
}

double PID::TotalError(double speed) {
  return (Kp - 0.0035 * speed) * p_error + Ki * i_error + (Kd + 0.0003 * speed) * d_error;
}

double PID::add_err(double err){
  i = (i+1) % DEQUE_LIMIT ;
  sum = sum - deque_elems[i] + err ;
  return sum;
}