#ifndef __PID_HPP__
#define __PID_HPP__

class PID{
    public:
        PID();
        float pos_type_pid(); // 位置型PID
        float vel_type_pid(); // 速度型PID
        float pi_d();         // 微分先行型PID
        float i_pd();         // 比例微分先行型PID
    private: 
        float error;
        float set_point;
        float actual;
        float integral;
        float dt;
        float deriv;
        float output;
};

#endif // __PID_HPP__