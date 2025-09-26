#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doStop();
        void doCount();
        void doReset();
        void doPeriod();
        void updateValueSliderLabel(int value);
        void changeSense();

    private:
        QTimer timer;
        int count;
        bool sense;
};

#endif // ejemplo1_H

/*
#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

class Timer
{
    public:

        Timer(){};

        template <class callable>
        void connect(callable&& f)
        {
			std::thread([=]()
            {
                while(true)
                {
					if(go.load())
						std::invoke(f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
                }
            }).detach();
        };

        void start(int p)
        {
			period.store(p);
			go.store(true);
        };

        void stop() { go.store(!go); };
		void setPeriod(int p) { period.store(p) ;};

    private:
        std::atomic_bool go = false;
		std::atomic_int period = 0;


};

#endif // TIMER_H
*/