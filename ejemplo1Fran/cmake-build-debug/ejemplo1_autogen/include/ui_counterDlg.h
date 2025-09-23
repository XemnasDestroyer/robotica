/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *StopButton;
    QLCDNumber *lcdNumber;
    QPushButton *ResetButton;
    QSlider *verticalSlider;
    QLabel *milisegundos;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName("Counter");
        Counter->resize(567, 300);
        StopButton = new QPushButton(Counter);
        StopButton->setObjectName("StopButton");
        StopButton->setGeometry(QRect(80, 180, 251, 71));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName("lcdNumber");
        lcdNumber->setGeometry(QRect(50, 40, 301, 91));
        ResetButton = new QPushButton(Counter);
        ResetButton->setObjectName("ResetButton");
        ResetButton->setGeometry(QRect(400, 40, 141, 71));
        verticalSlider = new QSlider(Counter);
        verticalSlider->setObjectName("verticalSlider");
        verticalSlider->setGeometry(QRect(380, 130, 31, 161));
        verticalSlider->setOrientation(Qt::Vertical);
        milisegundos = new QLabel(Counter);
        milisegundos->setObjectName("milisegundos");
        milisegundos->setGeometry(QRect(440, 190, 66, 18));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QCoreApplication::translate("Counter", "Counter", nullptr));
        StopButton->setText(QCoreApplication::translate("Counter", "STOP", nullptr));
        ResetButton->setText(QCoreApplication::translate("Counter", "Reset", nullptr));
        milisegundos->setText(QCoreApplication::translate("Counter", "    ms", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
