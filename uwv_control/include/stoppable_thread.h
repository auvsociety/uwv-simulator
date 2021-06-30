#ifndef STOPPABLE_THREAD_H
#define STOPPABLE_THREAD_H

#include <thread>
#include <assert.h>
#include <chrono>
#include <future>

/**
 * @brief Abstract class that encapsulates promise and future object 
 *        and provides API to set exit signal for the thread.
 * 
 * Reference: https://thispointer.com/c11-how-to-stop-or-terminate-a-thread/
 */
class StoppableThread
{
    std::promise<void> exit_signal_;
    std::future<void> future_obj_;
public:

    StoppableThread() :
            future_obj_(exit_signal_.get_future())
    {
    }

    StoppableThread(StoppableThread && obj) : exit_signal_(std::move(obj.exit_signal_)), future_obj_(std::move(obj.future_obj_))
    {
    }

    StoppableThread & operator=(StoppableThread && obj)
    {
        exit_signal_ = std::move(obj.exit_signal_);
        future_obj_ = std::move(obj.future_obj_);
        return *this;
    }

    /**
     * @brief Definition to this function must be provided in the derived class.
     * It will be called by thread function.
     */ 
    virtual void run() = 0;

    /**
     * @brief Thread function to be executed by thread.
     */ 
    void operator()()
    {
        run();
    }

    /**
     * @brief Checks if thread is requested to stop.
     */
    bool stopRequested()
    {
        // checks if value in future object is available
        if (future_obj_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout)
            return false;
        return true;
    }

    /**
     * @brief Request the thread to stop by setting value in promise object.
     */
    void stop()
    {
        exit_signal_.set_value();
    }

    /**
     * @brief Resets the exit signal to reuse the stoppable thread. This must be called only after stop()
     */ 
    void reuseThread(void){
        std::promise<void> es;
        exit_signal_ = std::move(es);
        future_obj_ = exit_signal_.get_future();
    }
};

#endif