#include "ejemplo1.h"
#include "QTimer"

ejemplo1::ejemplo1(): Ui_Counter()
{
	cont = 0;
	setupUi(this);
	show();
	connect(stopButton, SIGNAL(clicked()), this, SLOT(doStopButton()) );

	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.start(1000);

	connect(resetButton, SIGNAL(clicked()), this, SLOT(doResetButton()) );

	connect(verticalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	verticalSlider->setRange(0, 100);
}

void ejemplo1::doStopButton()
{
	if (timer.isActive())
		timer.stop();
	else
		timer.start(1000);
}

void ejemplo1::doCount()
{
	lcdNumber->display(cont++);
}

void ejemplo1::doResetButton()
{
	cont = 0;
	lcdNumber->display(0);
}

void ejemplo1::doSlider(int interval)
{
	QString text = QString::number(interval) + "%";
	label->setText(text);
	timer.setInterval(1000 - interval*10);
}

