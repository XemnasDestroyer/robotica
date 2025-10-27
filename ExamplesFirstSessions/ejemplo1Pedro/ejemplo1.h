#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>

#include "timer.h"
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doStopButton();
        void doCount();
        void doResetButton();
        void doSlider(int);

    private:
        int cont;
        QTimer timer;
        Timer myTimer;
};

#endif // ejemplo1_H
