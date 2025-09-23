#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "Timer.h"
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void stopButton();
        void doCount();
        void resetButton();
        void adjustSpeed(int value);

    private:
        Timer timer;
        bool contando;
        int contador;
};

#endif // ejemplo1_H
