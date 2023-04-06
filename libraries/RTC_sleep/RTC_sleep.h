

#ifndef RTC_sleep_h
#define RTC_sleep_h

class RTC_sleep
{
    public:
        void InitRTCInt();
        void MCUsleep(int downtime);
};

#endif