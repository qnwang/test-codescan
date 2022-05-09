/*
 * File:   timer_poll.cpp
 * Author: Administrator
 */
#include <cstdlib>
#include"EV_Timer.h"
#include"TimersPoll.h"
using namespace std;

int TimersPoll::timers_poll_add_timer(EV_Timer& ptimer) {
	int timer_id = ptimer.timer_get_id();
	struct epoll_event ev;
	ev.data.fd = timer_id;
	ev.events = EPOLLIN | EPOLLET;
	timers_map[timer_id] = ptimer; //add or modify
	epoll_ctl(epfd, EPOLL_CTL_ADD, timer_id, &ev);
	ptimer.timer_start();

	return 0;
}
int TimersPoll::timers_poll_del_timer(EV_Timer& ptimer) {
	int timer_id = ptimer.timer_get_id();
	struct epoll_event ev;
	ev.data.fd = timer_id;
	ev.events = EPOLLIN | EPOLLET;
	epoll_ctl(epfd, EPOLL_CTL_DEL, timer_id, &ev);
	timers_map.erase(timer_id);

	return 0;
}

int TimersPoll::timers_poll_judge_exist(EV_Timer& ptimer)
{
	int timer_id = ptimer.timer_get_id();
//	struct epoll_event ev;
//	ev.data.fd = timer_id;
//	ev.events = EPOLLIN | EPOLLET;
//	epoll_ctl(epfd, EPOLL_CTL_DEL, timer_id, &ev);
//	timers_map.erase(timer_id);
	int ret = timers_map.count(timer_id);
	return ret;
}

int TimersPoll::loop() {
	char buf[128] = { 0 };
	for (; active;) {
		struct epoll_event events[MAXFDS] = { 0 };
		int nfds = epoll_wait(epfd, events, MAXFDS, -1);
		for (int i = 0; i < nfds; ++i) {
		    std::map<int, EV_Timer>::iterator itmp = timers_map.find(
					events[i].data.fd);
			if (itmp != timers_map.end()) {
				//timer ptimer = itmp->second;
				while (read(events[i].data.fd, buf, 128) > 0)
					;
				itmp->second.runCallback();
			}
		}
	}
}

int TimersPoll::timers_poll_modify_value(EV_Timer &ptimer,double timer_value){
    ptimer.timer_modify_value(timer_value);
}

int TimersPoll::timers_poll_modify_internal(EV_Timer &ptimer,double timer_internal)
{
	ptimer.timer_modify_internal(timer_internal);
	return 0;
}
int TimersPoll::timers_poll_modify_cb(EV_Timer &ptimer,timer_callback callbk)
{
	ptimer.timer_modify_cb(callbk);
	return 0;
}
