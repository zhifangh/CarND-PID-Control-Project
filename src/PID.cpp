#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{
    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;

    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;

    this->privious_cte = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;

    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;

    this->privious_cte = 0;
    m_dTotalError = 0;
    m_uiTimes = 0;
}

void PID::UpdateError(double cte)
{
    /**
   * TODO: Update PID errors based on cte.
   */

    this->p_error = cte;
    this->i_error += cte;
    this->d_error = cte - this->privious_cte;

    this->privious_cte = cte;

    m_dTotalError += cte * cte;
    m_uiTimes++;
}

double PID::TotalError()
{
    /**
   * TODO: Calculate and return the total error
   */
  
//     Ki = 0;

    double dTotalError = Kp * p_error + Ki * i_error + Kd * d_error;

    return dTotalError;
}

double PID::GetSumError()
{
    return m_dTotalError;
}

double PID::GetAverageError()
{
    if (m_uiTimes > 0)
    {
        return m_dTotalError / m_uiTimes;
    }

    return 0;
}
