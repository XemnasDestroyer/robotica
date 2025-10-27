#include "ejemplo1.h"
#include "qtimer.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	count = 0;
    sense = true;
	connect(stop, SIGNAL(clicked()), this, SLOT(doStop()) );
	connect(reset, SIGNAL(clicked()), this, SLOT(doReset()) );
    connect(reverse, SIGNAL(clicked()), this, SLOT(changeSense()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	connect(slider, SIGNAL(valueChanged(int)), this, SLOT(doPeriod()) );
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(updateValueSliderLabel(int)) );
	timer.start(500);
}

void ejemplo1::doStop()
{
	if (timer.isActive()) {
		timer.stop();
	} else {
		timer.start(500);
	}
}

void ejemplo1::doCount()
{
    if(sense)
        lcdNumber->display(count++);
    else
        lcdNumber->display(count--);
}

void ejemplo1::doReset()
{
	count = 0;
	lcdNumber->display(count);
}

void ejemplo1::doPeriod()
{
    timer.setInterval(slider->maximum() - slider->value());
}

void ejemplo1::updateValueSliderLabel(int value)
{
    label->setText(QString::number(value) + "m/s");
}

void ejemplo1::changeSense()
{
    sense = !sense;
}




