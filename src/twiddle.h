#ifndef _TWIDDLE_H_
#define _TWIDDLE_H_

#include <vector>


class Twiddle
{
public:
    Twiddle();
    void Init();
    void Update(int iTimes, double iError);
    double CalculateTolerance();

public:
    std::vector<double> m_vParams;
    std::vector<double> m_vDParams;
    unsigned int m_uiParamNum;

public:
    double m_dBestError;
    double m_dTolerance;
};

#endif // _TWIDDLE_H_
