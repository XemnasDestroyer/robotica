#include "ejemplo1.h"
#include "QTimer"

ejemplo1::ejemplo1(): Ui_Counter()
{
	contador=0;
	contando=true;
	setupUi(this);
	show();

	verticalSlider->setMinimum(1);
	verticalSlider->setMaximum(2000);
	verticalSlider->setValue(1000);
	verticalSlider->setInvertedAppearance(true);
	milisegundos->setText(QString("%1 ms").arg(verticalSlider->value()));
	connect(StopButton, SIGNAL(clicked()), this, SLOT(stopButton()) );
	timer.connect(std::bind(&ejemplo1::doCount, this));
	connect(ResetButton, SIGNAL(clicked()), this, SLOT(resetButton()) );
	connect(verticalSlider, SIGNAL(valueChanged(int)), this, SLOT(adjustSpeed(int)));
	timer.start(1000);
}

void ejemplo1::stopButton()
{
	contando= !contando;

	if (contando) {
		qDebug() << "Contador reanudado";
		StopButton->setText("STOP");
	} else {
		qDebug() << "Contador detenido";
		StopButton->setText("START");
	}
}

void ejemplo1::resetButton()
{
    contador=0;
    lcdNumber->display(0);
	contando=false;
	StopButton->setText("START");
}

void ejemplo1::adjustSpeed(int value) {

	timer.setInterval(value);
	milisegundos->setText(QString("%1 ms").arg(value));
}

void ejemplo1::doCount()
{
	if(contando) {
		lcdNumber->display(contador++);
	}
}


