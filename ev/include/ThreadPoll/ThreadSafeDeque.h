//
// Created by jim on 9/19/18.
//

#ifndef IPC_SERVICE_THREADSAFEDEQUE_H
#define IPC_SERVICE_THREADSAFEDEQUE_H
#include <deque>
#include <mutex>
#include <condition_variable>

template <typename T>
class thread_safe_deque
{
public:
    thread_safe_deque() = default;
    thread_safe_deque(const thread_safe_deque&) = delete;
    thread_safe_deque& operator=(const thread_safe_deque&) = delete;
    ~thread_safe_deque() = default;

    void enDeque(T& data){
        std::unique_lock<std::mutex> lk(mtx);
        deque.emplace_back(data);
//        lk.unlock();
        cond_read.notify_all();
    }

    void deDeque(T& buf) {
        std::unique_lock<std::mutex> lk(mtx);
        cond_read.wait(lk,[this](){return !deque.empty();});
        buf = std::move(deque.front());
        deque.pop_front();
//        lk.unlock();
        cond_write.notify_all();
    }

    size_t size(){
        std::unique_lock<std::mutex> lk(mtx);
        return deque.size();
    }

    bool empty(){
        std::unique_lock<std::mutex> lk(mtx);
        return deque.empty();
    }

    bool clear(){
        std::unique_lock<std::mutex> lk(mtx);
        deque.clear();
        return !deque.size();
    }

    std::mutex& get_read_mtx(){return m_read;}

    std::mutex& get_write_mtx(){return m_write;}
private:
    std::deque<T> deque;
    std::mutex mtx, m_read, m_write;
    std::condition_variable cond_read, cond_write;
};

#endif //IPC_SERVICE_THREADSAFEDEQUE_H
