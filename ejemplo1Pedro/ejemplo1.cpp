#include "ejemplo1.h"
#include "QTimer"

ejemplo1::ejemplo1(): Ui_Counter()
{
	cont = 0;
	setupUi(this);
	show();
	connect(stopButton, SIGNAL(clicked()), this, SLOT(doStopButton()) );

	//TIMER DE Qt
	/*connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.start(1000);*/

	//TIMER DE C++
	myTimer.connect(std::bind(&ejemplo1::doCount, this));
	myTimer.start(1000);

	connect(resetButton, SIGNAL(clicked()), this, SLOT(doResetButton()) );

	connect(verticalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	verticalSlider->setRange(0, 100);
}

void ejemplo1::doStopButton()
{
	//PARA TIMER DE Qt
	if (timer.isActive())
		timer.stop();
	else
		timer.start(1000);

	//PARA TIMER DE C++
	myTimer.stop();
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

	//CAMBIA LA FRECUENCIA DEL TIMER DE Qt
	timer.setInterval(1000 - interval*10);

	//CAMBIA LA FRECUENCIA DEL TIMER DE C++
	myTimer.setPeriod(1000 - interval*10);
}

