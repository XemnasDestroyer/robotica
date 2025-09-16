#include "ejemplo1.h"
#include "QTimer"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.start(1000);
}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
}

void ejemplo1::doCount()
{
	static int cont = 0;
	lcdNumber->display(cont++);
}


