
#include"EV_Timer.h"

using namespace std;

EV_Timer::EV_Timer(const EV_Timer& ptimer) {
	timer_internal = ptimer.timer_internal;
	timer_value = ptimer.timer_value;
	cb = ptimer.cb;
	timer_id = ptimer.timer_id;
}
EV_Timer & EV_Timer::operator =(const EV_Timer& ptimer) {
	if (this == &ptimer) {
		return *this;
	}
	timer_internal = ptimer.timer_internal;
	timer_value = ptimer.timer_value;
	cb = ptimer.cb;
	timer_id = ptimer.timer_id;
	return *this;
}
int EV_Timer::timer_start() {
	struct itimerspec ptime_internal = { 0 };
	ptime_internal.it_value.tv_sec = (int) timer_value;
	ptime_internal.it_value.tv_nsec = (timer_value - (int) timer_value) * 1000000;
	if (timer_internal!=0) {
		ptime_internal.it_interval.tv_sec = (int) timer_internal;
		ptime_internal.it_interval.tv_nsec = (timer_internal - (int) timer_internal)* 1000000;
	}

	timerfd_settime(timer_id, 0, &ptime_internal, NULL);
	return 0;
}
int EV_Timer::timer_stop() {
	close(timer_id);
	return 0;
}

int EV_Timer::timer_modify_value(double timer_value){
    this->timer_value = timer_value;
    timer_start();
}

int EV_Timer::timer_modify_internal(double timer_internal) {
	this->timer_internal = timer_internal;
	timer_start();
}
int EV_Timer::timer_modify_cb(timer_callback callbk){
	this->cb = callbk;
	timer_start();
}



