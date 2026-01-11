#ifndef WAITING_H
#define WAITING_H

#include <QLabel>
#include <QWidget>
#include <QTimer>
#include <QTimer>

class QMovie;

namespace Ui {
class waiting;
}

class waiting : public QWidget
{
    Q_OBJECT

public:
    explicit waiting(QWidget *parent = 0);
    ~waiting();
public slots:


signals:
    void loading();

private:
    Ui::waiting *ui;
    QMovie *movie;
    QLabel *label;
    QLabel * tip_label;
    QFrame * background;
    QTimer *timer ;
};

#endif // WAITING_H
