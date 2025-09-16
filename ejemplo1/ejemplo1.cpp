#include "ejemplo1.h"
#include "qtimer.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.start(500);
}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
}

void ejemplo1::doCount()
{
	static int count = 0;
	lcdNumber->display(count++);
}




