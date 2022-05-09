#ifndef SERVICE_THREADSAFEQUEUE_H
#define SERVICE_THREADSAFEQUEUE_H
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class thread_safe_queue
{
public:
    thread_safe_queue() = default;
    thread_safe_queue(const thread_safe_queue&) = delete;
    thread_safe_queue& operator=(const thread_safe_queue&) = delete;
    ~thread_safe_queue() = default;

    void enQueue(T& data){
        std::unique_lock<std::mutex> lk(mtx);
        queue.push(data);
//        lk.unlock();
        cond_read.notify_all();
    }

    void deQueue(T& buf) {
        std::unique_lock<std::mutex> lk(mtx);
        cond_read.wait(lk,[this](){return !queue.empty();});
        buf = std::move(queue.front());
        queue.pop();
//        lk.unlock();
        cond_write.notify_all();
    }

    size_t size(){
        std::unique_lock<std::mutex> lk(mtx);
        return queue.size();
    }

    bool empty(){
        std::unique_lock<std::mutex> lk(mtx);
        return queue.empty();
    }

    std::mutex& get_read_mtx(){return m_read;}

    std::mutex& get_write_mtx(){return m_write;}
private:
    std::queue<T> queue;
    std::mutex mtx, m_read, m_write;
    std::condition_variable cond_read, cond_write;
};

#endif //SERVICE_THREADSAFEQUEUE_H
