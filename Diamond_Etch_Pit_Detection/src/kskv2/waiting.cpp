#include "waiting.h"
#include "ui_waiting.h"
#include <qmovie.h>
waiting::waiting(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::waiting)
{
    ui->setupUi(this);
    this->setFixedSize(200,200);
    background = new QFrame(this);
    background->setStyleSheet("background-color:#fff;border-radius:10px;");
    background->setGeometry(0, 0, 200, 200);
    label = new QLabel(background);
    label->setGeometry(0, 0, 200, 200);
    movie = new QMovie("loading.gif");
    movie->setScaledSize(QSize(200,200));
    label->setScaledContents(true);
    label->setMovie(movie);
    movie->start();
    qDebug()<<"loading";
    connect(this,SIGNAL(loading()),this,SLOT(close()));
}


waiting::~waiting()
{
//一定要delete界面元素,不然会留下现一个透明方框
    delete background;
    delete label;
    delete movie;
    delete ui;
}
