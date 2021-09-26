#include <assert.h>
#include <iostream>

#include "twiddle.h"

Twiddle::Twiddle()
{
    Init();
}

void Twiddle::Init()
{
    m_vParams.clear();
    m_vDParams.clear();

    m_vParams.push_back(0);
    m_vParams.push_back(0);
    m_vParams.push_back(0);

    m_vDParams.push_back(1);
    m_vDParams.push_back(1);
    m_vDParams.push_back(1);

    m_uiParamNum = m_vParams.size();

    m_dTolerance = 0.2;
    m_dBestError = 10000;
}

void Twiddle::Update(int iTimes, double iError)
{
    static int indexParam = 0;
    static int loopTimes = 0;
    if (CalculateTolerance() > m_dTolerance)
    {

        if (indexParam >= m_vParams.size())
        {
            indexParam = 0;
        }

        std::cout << "Best Error: " << m_dBestError << std::endl;
        std::cout << "kp: " << m_vParams[0] << " ki: " << m_vParams[1] << " kd: " << m_vParams[2]<< std::endl;
        std::cout << "dp: " << m_vDParams[0] << " di: " << m_vDParams[1] << " dd: " << m_vDParams[2]<< std::endl;

//        std::cout << "step1" << std::endl;
        std::cout << "indexParam: " << indexParam << std::endl;
        std::cout << "loopTiles: " << loopTimes << std::endl;

      while (indexParam < m_vParams.size())
      {

      
        if (0 == loopTimes)
        {
            std::cout << "step2" << std::endl;
            m_vParams[indexParam] += m_vDParams[indexParam]; // first time : + dp

            loopTimes++; // change p, then try it
            return;
        }


        if (1 == loopTimes)
        {
//            std::cout << "step3" << std::endl;
            if (iError < m_dBestError)
            {
                std::cout << "step4" << std::endl;
                m_dBestError = iError;
                m_vDParams[indexParam] *= 1.1; // increate dp

                ++indexParam; // process next p
                loopTimes = 0;
              continue;
//                 return;
            }
            else
            {
                std::cout << "step5" << std::endl;
                m_vParams[indexParam] -= 2 * m_vDParams[indexParam]; // second time: - 2 * dp

                loopTimes++; // try again
                return;
            }
        }

        if (2 == loopTimes)
        {
//            std::cout << "step6" << std::endl;
            if (iError < m_dBestError)
            {
                std::cout << "step7" << std::endl;
                m_dBestError = iError;
                m_vDParams[indexParam] *= 1.1; // increate dp

                ++indexParam; // process next p
                loopTimes = 0;
              continue;
//                 return;
            }
            else
            {
                std::cout << "step8" << std::endl;
                m_vParams[indexParam] += m_vDParams[indexParam]; // third time : store p,  + dp - 2 * dp + dp
                m_vDParams[indexParam] *= 0.9; // decreate dp, 

                ++indexParam; // process next p
                loopTimes = 0;
              continue;
//                 return;
            }
        }
      };
    }

    std::cout << "step9" << std::endl;

    std::cout << "kp:" << m_vParams[0] << std::endl;
    std::cout << "ki:" << m_vParams[1] << std::endl;
    std::cout << "kd:" << m_vParams[2] << std::endl;

    assert(0);
}

double Twiddle::CalculateTolerance()
{
    double tol = 0;
    for (auto it = m_vDParams.begin(); it != m_vDParams.end(); ++it)
    {
        tol += *it;
    }

    return tol;
}
