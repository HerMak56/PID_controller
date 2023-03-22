#pragma once

class Motor
{
    private:
        // Val for pins
        int FirstEncoder;
        int SecondEncoder;
        int PWMOut;
        int RotOut1;
        int RotOut2;

        //Val for flags
        bool FlagInterrapt;

        //Val for calculate vel
        int count;
        int count_prev; // for calculate delta
        float delta;
        float Velocity;
        float GoalVelocity;

        //Timer
        uint32_t Timer;

        //Val for state Encoders
        bool StateFirstEncoder;
        bool StateSecondEncoder;

        //Val for PID
        double error = 0;
        double integral = 0;
        double prev_error = 0;
        double D = 0;
        double kp = 0.75; // 0.75 0.5 0
        double ki = 1;
        double kd = 0 ;
        double out = 0;

        //Val for filter
        float _err_measure;
        float _q;


        void FlagInterrupt();
        void Send2Driver(float V);
        void StatsFlag();
        void calculateRotSpeed();
        void VelocityPID(double GoalVelocity, double Velocity);
        float simpleKalman(float newVal);

        public:

        void init(int FirstEncoder,int SecondEncoder, int PWMOut, int RotOut1, int RotOut2);
        void tick();
        void SetVolocity(float GoalVelocity);
        float GetVelocity();
};