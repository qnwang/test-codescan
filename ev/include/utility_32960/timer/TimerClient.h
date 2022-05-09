#ifndef TIMERCLIENT
#define TIMERCLIENT

#include"EV_Timer.h"
#include"TimersPoll.h"
class TimerClient {
public:

    virtual ~TimerClient();
    static 	std::shared_ptr<TimerClient> getInstance();
    // Timer
    int add_timer(EV_Timer &ptimer);
    int del_timer(EV_Timer &ptimer);
    int chekExist_timer(EV_Timer &ptimer);
    int mod_timer_value(EV_Timer &ptimer,double timer_value);
    int mod_timer_internal(EV_Timer &ptimer,double timer_internal);
    int mod_timer_func_cb(EV_Timer &ptimer,timer_callback callbk);
    int deactive_timerLoop();

private:
    TimerClient();
    TimersPoll m_timers;
    static std::shared_ptr<TimerClient> pInstance;
};



#endif /* TIMERCLIENT*/
