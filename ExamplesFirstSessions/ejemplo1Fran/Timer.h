#ifndef EJEMPLO1_TIMER_H
#define EJEMPLO1_TIMER_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>

class Timer {
public:
    Timer() : go(false), period(0) {}

    template <class T>
    void connect(T f)
    {
        std::thread([this, f = std::move(f)]()
        {
            while(true)
            {
                if(go.load())
                    std::invoke(f);
                std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
            }
        }).detach();
    }

    void start(int p)
    {
        period.store(p);
        go.store(true);
    }

    void stop()
    {
        go.store(false);
    }

    void setInterval(int p)
    {
        period.store(p);
    }

private:
    std::atomic_bool go;
    std::atomic_int period;
};


#endif