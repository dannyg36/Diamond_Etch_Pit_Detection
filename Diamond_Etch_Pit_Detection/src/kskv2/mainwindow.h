#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include<qfiledialog.h>
#include<qstring.h>
#include <QDirIterator>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <time.h>
#include <sys/syscall.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <QMessageBox>
#include "MyLabel.h"
#include "waiting.h"
#include <thread>
#include <QTextBrowser>
using namespace std;
using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct HeatmapPoint {
  int x;
  int y;
  double denskty;
  int weight;
};

struct HeatmapRect {
  int left;
  int top;
  int right;
  int bottom;
};
struct KskHeatmapPara {
    vector<HeatmapPoint> points;
    vector<HeatmapRect> areas;
    double max_density;
    int reach;
    array<array<uchar, 3>, 256> color_table;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    waiting *ptrLoadingDialog;
    void KSKCalculator();
    void KSK10x10Calculator();
    void showLoadingDialog();
    void OpenKSKFile();
    void doAppendText(QTextBrowser *obj, const QString &text);
    void doShowImage(const QImage &image, const QString &title);
signals:
    void send2Loading();
    void AppendText(QTextBrowser *obj, const QString &text);
    void ShowImage(const QImage &image, const QString &title);
private slots:
    void on_btn_open_src_clicked();
    void StartPointSlot(QPoint p);
    void StopPointSlot(QPoint p);

    void on_btn_show_capture_clicked();

    void on_btn_cal_ksk_clicked();

    void on_btn_demarcate_clicked();

    void on_btn_10x10_cal_ksk_clicked();
    void SlotAppendText(QTextBrowser *obj, const QString &text);
    void SlotShowImage(const QImage &image, const QString &title);
protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    Ui::MainWindow *ui;
    QString mPicFilepath;
    MyLabel* mPreviewLabel;
    Mat mRoiImage;
    Mat mSrcImage;
    Mat mPreviewSrc;
    QPoint mStartPoint;
    int mRoiX;
    int mRoiY;
    int mRoiW;
    int mRoiH;
    int mDeepNumber;
    int mShallowNumber;

    void resetUI();
    int analysis(Mat input, bool cal_deep_shallow = false);

    int mDivPreview = 12;
    int mDiv10x10 = 20;

    QLabel* labelROI;
    QLabel* label10x10;
    QLabel* label10x10Heatmap;
    //KskHeatmapPara m10x10HeatmapPara;
    //生成渐变色
    void Gradient(array<uchar, 3> &start, array<uchar, 3> &end,
                  vector<array<uchar, 3>> &RGBList) {
      array<float, 3> dBgr;
      for (int i = 0; i < 3; i++) {
        dBgr[i] = (float)(end[i] - start[i]) / (RGBList.size() - 1);
      }

      for (size_t i = 0; i < RGBList.size(); i++) {
        for (int j = 0; j < 3; j++) {
          RGBList[i][j] = (uchar)(start[j] + dBgr[j] * i);
        }
      }
    };
    void InitAlpha2ColorTable(array<array<uchar, 3>, 256> &table) {
      array<double, 7> boundaryValue = {0.2, 0.3, 0.4, 0.6, 0.8, 0.9, 1.0};
      array<array<uchar, 3>, 7> boundaryBGR;
      boundaryBGR[0] = {255, 0, 0};
      boundaryBGR[1] = {231, 111, 43};
      boundaryBGR[2] = {241, 192, 2};
      boundaryBGR[3] = {148, 222, 44};
      boundaryBGR[4] = {83, 237, 254};
      boundaryBGR[5] = {50, 118, 253};
      boundaryBGR[6] = {28, 64, 255};

      double lastValue = 0;
      array<uchar, 3> lastRGB = {0, 0, 0};
      vector<array<uchar, 3>> RGBList;
      int sumNum = 0;
      for (size_t i = 0; i < boundaryValue.size(); i++) {
        int num = 0;
        if (i == boundaryValue.size() - 1) {
          num = 256 - sumNum;
        } else {
          num = (int)((boundaryValue[i] - lastValue) * 256 + 0.5);
        }

        RGBList.resize(num);
        Gradient(lastRGB, boundaryBGR[i], RGBList);

        for (int i = 0; i < num; i++) {
          table[i + sumNum] = RGBList[i];
        }
        sumNum = sumNum + num;

        lastValue = boundaryValue[i];
        lastRGB = boundaryBGR[i];
      }
    };
};
#endif // MAINWINDOW_H
