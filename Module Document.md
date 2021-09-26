# CarND-PID-Control-Project

## **Goals**

The goals of this project are the following:

* Implement a PID controller in C++ to maneuver the vehicle around the track!

* The vehicle must successfully drive a lap around the track.

  

### Pipeline 

### Input

#### 1. CTE

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

```c++
      double cte = std::stod(j[1]["cte"].get<string>());
      double speed = std::stod(j[1]["speed"].get<string>());
      double angle = std::stod(j[1]["steering_angle"].get<string>());
```


## Theory

A proportional–integral–derivative controller (**PID controller** or three-term controller) is a control loop mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively), hence the name.



The PID formula is as follows:

![](D:\hzf\udacity\project\CarND-PID-Control-Project\image\ut.png)



Its discretization is expressed as follows：

![](D:\hzf\udacity\project\CarND-PID-Control-Project\image\uk.png)



Kp: proportionality coefficient

Ki = (Kp * T) / Ti : integral coefficient

Kd = (Kp * Td) / T : derivative coefficient



The formula can be expressed as the follows:

**u = Kp * p_error + Ki * i_error + Kd * d_error**



When the ego car moves with a constant velocity. The reference trajectory would be the x-axis. the y-axis will represent the distance between the ego car and the reference trajectory line.  Call this Cross-Track-Error (CTE in short).  CTE is the error which is processed by the PID controler in this project. 



## Implement

### 1. Init PID controler parameters:

```c++
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
```

### 2. Update the terms

At each iteration, Proportional term、integral term and derivative term are updated.

```c++
void PID::UpdateError(double cte)
{
    /**
   * TODO: Update PID errors based on cte.
   */

    this->p_error = cte;
    this->i_error += cte;
    this->d_error = cte - this->privious_cte;

    this->privious_cte = cte;

    m_dTotalError += cte;
    m_uiTimes++;
}
```

### 3. Caculate the result

Calculate the output based on the terms and their coefficients.

```c++
double PID::TotalError()
{
    /**
   * TODO: Calculate and return the total error
   */

    double dTotalError = Kp * p_error + Ki * i_error + Kd * d_error;

    return dTotalError;
}
```



## Tune Hyperparameters

### 1. Twiddle

To select parameters, I first tried the Twiddle scheme.  

```c++
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
            }
            else
            {
                std::cout << "step8" << std::endl;
                m_vParams[indexParam] += m_vDParams[indexParam]; // third time : store p, + dp - 2 * dp + dp
                m_vDParams[indexParam] *= 0.9; // decreate dp, 

                ++indexParam; // process next p
                loopTimes = 0;
              	continue;
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
```

 I adjust the cycle of iteration: 200 -> 100 -> 50 -> 30 -> 20 -> 10 -> 5 -> 3 -> 2

```c++
if (iTimes++ % 100 == 0)
{
    std::cout << "AverageError: " << pid.GetAverageError() << std::endl;
    twiddle.Update(0, pid.GetAverageError());

    std::cout << "Best Error: " << twiddle.m_dBestError << std::endl;
    std::cout << "kp: " << twiddle.m_vParams[0] << " ki: " << twiddle.m_vParams[1] << " kd: " << twiddle.m_vParams[2]<< std::endl;
    std::cout << "dp: " << twiddle.m_vDParams[0] << " di: " << twiddle.m_vDParams[1] << " dd: " << twiddle.m_vDParams[2]<< std::endl;

    pid.Init(twiddle.m_vParams[0], twiddle.m_vParams[1], twiddle.m_vParams[2]);
}
```

The effect is not ideal, as the following video shows. The CTE can't convergent and the ego car always run off the road: 

 <video src="D:\hzf\udacity\project\CarND-PID-Control-Project\vedio\twiddle_tune.mp4"></video>

I analyzed the possible reason: Using Twiddle to find the optimal result, need to base on a set of fixed initial conditions, compare the results using different parameters under the same initial conditions. However, in this project, the vehicle was moving in real time. After I adjusted the parameters through Twiddle, the running state of the vehicle had changed, and it was different  with the initial conditions in the last iteration process. Therefore, it was difficult to judge the quality of the selected parameters through the average error in the iteration cycle.  

### 2. Manual Tune

Since the project uses a simulated virtual environment, there is no system bias, I keep Ki at 0 and use PD Controler to achieve the project goals.  

I first try to adjust Kp, which is supposed to be a negative value, and I adjust Kd to stabilize the vehicle when it moves back and forth in the center of the lane. Finally, I got a group of paramter values [-0.20, 0.0, -2.5]. Using these parameters, the vehicle can run more smoothly near the center line of the lane. The effect is as follows: 

<video src="D:\hzf\udacity\project\CarND-PID-Control-Project\vedio\manual_tune.mp4"></video>

### Output

Finally, the steer value is sent to simulator.  

```c++
pid.UpdateError(cte);
steer_value = pid.TotalError();

// DEBUG
static int s_iTimes = 0;
std::cout << "[OUTPUT] Times: " << s_iTimes++ << " Steering Value: " << steer_value << std::endl;

json msgJson;
msgJson["steering_angle"] = steer_value;
msgJson["throttle"] = 0.3;
auto msg = "42[\"steer\"," + msgJson.dump() + "]";
std::cout << msg << std::endl;
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```
