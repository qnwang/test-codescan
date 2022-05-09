#ifndef TIMER_H_
#define TIMER_H_
#include <unistd.h>
#include <fcntl.h>
#include <sys/timerfd.h>
#include <functional>

typedef std::function<void()> timer_callback;
/*
 * timer_internal:间隔时间(0只执行一次)
 * timer_value：第一次启动时间(0不执行，此时timer_internal无效)
 */
class EV_Timer {
public:
    EV_Timer() :
		timer_value(0.0),timer_internal(0.0), cb(0){
		timer_id = timerfd_create(CLOCK_REALTIME, 0);
		setNonBlock(timer_id);
	}
	EV_Timer(double time_value,double internal_value, const timer_callback &callback) :
		timer_value(time_value),timer_internal(internal_value), cb(callback) {
		timer_id = timerfd_create(CLOCK_REALTIME, 0);
		setNonBlock(timer_id);
	}
	EV_Timer(const EV_Timer &ptimer);
    EV_Timer & operator=(const EV_Timer &ptimer);
	int timer_start();
	int timer_stop();
    int timer_modify_value(double timer_value);
	int timer_modify_internal(double timer_internal);
	int timer_modify_cb(timer_callback callbk);
	int timer_get_id() {
		return timer_id;
	}
	void runCallback() const{
		cb();
	}
	~EV_Timer() {
		timer_stop();
	}


private:
	bool setNonBlock(int fd) {
		int flags = fcntl(fd, F_GETFL, 0);
		flags |= O_NONBLOCK;
		if (-1 == fcntl(fd, F_SETFL, flags)) {
			return false;
		}
		return true;
	}
    double timer_value;
    double timer_internal;
    timer_callback cb;
    int timer_id;
};

#endif /* TIMER_H_ */
