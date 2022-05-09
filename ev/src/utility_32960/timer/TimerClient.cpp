#include <memory>
#include"TimerClient.h"

std::shared_ptr <TimerClient> TimerClient::pInstance(new TimerClient());

std::shared_ptr <TimerClient> TimerClient::getInstance() {
    return pInstance;
}

void *thread_fun(void *data) {
    TimersPoll *my_timers = static_cast<TimersPoll *>(data);
    my_timers->loop();
}


TimerClient::TimerClient() : m_timers() {
    pthread_t thread_id = 0;
    pthread_create(&thread_id, NULL, thread_fun, &m_timers);
}

TimerClient::~TimerClient() {
}

int TimerClient::add_timer(EV_Timer &ptimer) {
    return m_timers.timers_poll_add_timer(ptimer);
}

int TimerClient::del_timer(EV_Timer &ptimer) {
    return m_timers.timers_poll_del_timer(ptimer);
}

int TimerClient::mod_timer_value(EV_Timer &ptimer, double timer_value) {
    return m_timers.timers_poll_modify_value(ptimer, timer_value);
}

int TimerClient::mod_timer_internal(EV_Timer &ptimer, double timer_internal) {
    return m_timers.timers_poll_modify_internal(ptimer, timer_internal);
}

int TimerClient::mod_timer_func_cb(EV_Timer &ptimer, timer_callback callbk) {
    return m_timers.timers_poll_modify_cb(ptimer, callbk);
}

int TimerClient::deactive_timerLoop() {
    return m_timers.timers_poll_deactive();
}

int TimerClient::chekExist_timer(EV_Timer &ptimer) {
    return m_timers.timers_poll_judge_exist(ptimer);
}

