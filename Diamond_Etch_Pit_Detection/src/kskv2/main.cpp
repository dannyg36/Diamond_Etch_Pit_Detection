#include "mainwindow.h"

#include <QApplication>
#include <QStyleFactory>
int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    QApplication a(argc, argv);
    //QString qss;
    //QFile qssFile("myQss.qss");
    //qssFile.open(QFile::ReadOnly);

    //if(qssFile.isOpen())
    //{
    //    qss = QLatin1String(qssFile.readAll());
    //    qApp->setStyleSheet(qss);
    //    qssFile.close();
    //}
    MainWindow w;
    w.show();
    return a.exec();
}
