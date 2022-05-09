#ifndef TIMER_POLL_H
#define TIMER_POLL_H
#include <sys/epoll.h>
#include <map>

#define MAXFDS 128

class EV_Timer;

class TimersPoll {
public:
	TimersPoll(int max_num = 128) {
		active = 1;
		epfd = epoll_create(max_num);
	}
	int timers_poll_add_timer(EV_Timer &ptimer);
	int timers_poll_del_timer(EV_Timer &ptimer);
	int timers_poll_modify_value(EV_Timer &ptimer,double timer_value);
	int timers_poll_modify_internal(EV_Timer &ptimer,double timer_internal);
	int timers_poll_modify_cb(EV_Timer &ptimer,timer_callback callbk);
	int timers_poll_judge_exist(EV_Timer& ptimer);
	int loop();
	int timers_poll_deactive() {
		active = 0;
        return 1;
	}
	~ TimersPoll() {
	}
private:
	int epfd;
	int active;
	std::map<int, EV_Timer> timers_map;
	/* data */
};
#endif /* TIMER_POLL_H */
