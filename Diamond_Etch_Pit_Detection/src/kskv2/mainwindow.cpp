#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QObject>
#include <QThread>
#include <QRunnable>
#include <QThreadPool>
#include <QApplication>
#include <QDesktopWidget>
#include <QScreen>
using namespace std;
using namespace cv;
static MainWindow* ptrMainWindow = NULL;

class RunnableKSKCalculator : public QRunnable
{
    void run() {
        ptrMainWindow->KSKCalculator();
        usleep(500*1000);
        ptrMainWindow->ptrLoadingDialog->close();
    }
};

class RunnableKSK10x10Calculator : public QRunnable
{
    void run() {
        ptrMainWindow->KSK10x10Calculator();
        usleep(500*1000);
        ptrMainWindow->ptrLoadingDialog->close();
    }
};

class RunnableOpenKSKFile : public QRunnable
{
    void run() {
        ptrMainWindow->OpenKSKFile();
        usleep(500*1000);
        ptrMainWindow->ptrLoadingDialog->close();
    }
};

void MainWindow::doShowImage(const QImage &image, const QString &title) {
    emit ShowImage(image, title);
}
void MainWindow::SlotShowImage(const QImage &image, const QString &title) {
    QLabel* label_img = new QLabel(this, Qt::Dialog |Qt::WindowCloseButtonHint); /* 去掉?按钮 */
    label_img->setWindowTitle(title);
    //label_img->installEventFilter(this);

    label_img->setMaximumWidth(image.width());
    label_img->setMaximumHeight(image.height());
    label_img->resize(QSize(image.width(), image.height()));
    label_img->setPixmap(QPixmap::fromImage(image.rgbSwapped()));
    label_img->show();
}

void MainWindow::doAppendText(QTextBrowser *obj, const QString &text) {
    emit AppendText(obj, text);
}
void MainWindow::SlotAppendText(QTextBrowser *obj, const QString &text) {
    obj->setText(text);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mPreviewLabel = new MyLabel(this);

    mPreviewLabel->setStyleSheet("border:1px solid rgb(0, 0, 0)");
    mPreviewLabel->setMaximumSize(650,650);
    mPreviewLabel->setMinimumSize(650,650);
    ui->layout_imgpreview->addWidget(mPreviewLabel);
    connect(this,SIGNAL(AppendText(QTextBrowser *,QString)),this,SLOT(SlotAppendText(QTextBrowser *,QString)));
    connect(this,SIGNAL(ShowImage(QImage,QString)),this,SLOT(SlotShowImage(QImage,QString)));

    ptrMainWindow = this;

    //ui->btn_demarcate->setStyleSheet("background-color: rgb(255, 0, 0);");
}

MainWindow::~MainWindow()
{
    delete ui;
    ptrMainWindow = NULL;
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(obj == labelROI) {
        if(event->type() == QEvent::Close) {
            labelROI->removeEventFilter(this);
            qDebug() << "delete labelROI";

            labelROI->deleteLater();
            labelROI  = nullptr;
        }
    } else if(obj == label10x10 || obj == label10x10Heatmap) {
        if(event->type() == QEvent::Close) {
            obj->removeEventFilter(this);
            qDebug() << "delete label10x10";

            obj->deleteLater();
            obj  = nullptr;
        }
    }

    return QMainWindow::eventFilter(obj, event); /* 将事件传递给父类 */
}

void MainWindow::StartPointSlot(QPoint p) {
    //计算出原图的起始坐标
    mStartPoint = p;
    mRoiX = (p.x() * mPreviewSrc.cols) / mPreviewLabel->size().width();
    mRoiY = (p.y() * mPreviewSrc.rows) / mPreviewLabel->size().height();

    mRoiX = (mRoiX * mSrcImage.cols) / mPreviewSrc.cols;
    mRoiY = (mRoiY * mSrcImage.rows) / mPreviewSrc.rows;

    ui->text_select_x->setText(QString::number(mRoiX));
    ui->text_select_y->setText(QString::number(mRoiY));

}
void MainWindow::StopPointSlot(QPoint p) {
    //计算出对应原图的选框宽和高
    mRoiW = ((p.x() - mStartPoint.x()) * mPreviewSrc.cols) / mPreviewLabel->size().width();
    mRoiH = ((p.y() - mStartPoint.y()) * mPreviewSrc.rows) / mPreviewLabel->size().height();

    mRoiW = (mRoiW * mSrcImage.cols) / mPreviewSrc.cols;
    mRoiH = (mRoiH * mSrcImage.rows) / mPreviewSrc.rows;

    ui->text_select_w->setText(QString::number(mRoiW));
    ui->text_select_h->setText(QString::number(mRoiH));


}

void MainWindow::resetUI() {
    destroyAllWindows();
    ui->text_src_path->clear();
    ui->text_ksk_num->clear();
    ui->text_select_x->clear();
    ui->text_select_y->clear();
    ui->text_select_w->clear();
    ui->text_select_h->clear();
    ui->text_roi_pixel_area->clear();
    ui->text_roi_real_area->clear();
    ui->text_ksk_density->clear();
    ui->text_deep_num->clear();
    ui->text_shallow_num->clear();

    mRoiX = 0;
    mRoiY = 0;
    mRoiW = 0;
    mRoiH = 0;
}

void MainWindow::on_btn_open_src_clicked()
{
    //打开一个文件

    resetUI();
    mPicFilepath = QFileDialog::getOpenFileName(this, tr("打开文件"), 0);
    if(mPicFilepath.isEmpty()) {
        QMessageBox::warning(this,tr("Error"),tr("请选择正确的文件。"));
        return;
    }
    showLoadingDialog();
    RunnableOpenKSKFile* ptr = new RunnableOpenKSKFile();
    QThreadPool::globalInstance()->start(ptr);


}

void MainWindow::OpenKSKFile() {

    //ui->text_src_path->setText(mPicFilepath);
    doAppendText(ui->text_src_path, mPicFilepath);
    if(mPicFilepath.isEmpty()) {
        QMessageBox::warning(this,tr("Error"),tr("请选择正确的文件。"));
        return;
    }

    //关联开始坐标的信号
    connect(mPreviewLabel, SIGNAL(StartPointSignal(QPoint)), ptrMainWindow, SLOT(StartPointSlot(QPoint)));
    //关联结束坐标的信号
    connect(mPreviewLabel,SIGNAL(StopPointSignal(QPoint)), ptrMainWindow, SLOT(StopPointSlot(QPoint)));

    //显示图片预览图
    //mPreviewSrc = imread(mPicFilepath.toStdString().data());
    mSrcImage = imread(mPicFilepath.toLocal8Bit().data());
    if(mSrcImage.rows < 2000 || mSrcImage.cols < 2000) {
        mDivPreview = 1;
    }
    cv::Size dsize = cv::Size(mSrcImage.size().width/mDivPreview, mSrcImage.size().height/mDivPreview);
    cv::Mat resize_img;
    cv::resize(mSrcImage, mPreviewSrc, dsize, 0, 0, cv::INTER_AREA);

    printf("dbg>> image size is %dx%d\n", mPreviewSrc.cols, mPreviewSrc.rows);

    cvtColor(mPreviewSrc, mPreviewSrc, COLOR_BGR2RGB);
    QImage img = QImage((const unsigned char*)(mPreviewSrc.data),
                        mPreviewSrc.cols, mPreviewSrc.rows,
                        mPreviewSrc.cols*mPreviewSrc.channels(),
                        QImage::Format_RGB888).scaled(mPreviewLabel->width(), mPreviewLabel->height());

    mPreviewLabel->clear();
    mPreviewLabel->setPixmap(QPixmap::fromImage(img));
}

int MainWindow::analysis(Mat input, bool cal_deep_shallow) {
    int ksk_number = -1;
    cv::Mat src;

    mShallowNumber = 0;
    mDeepNumber = 0;

    cv::cvtColor(input, src, cv::COLOR_BGR2GRAY);
    //cv::imshow("src", src);


    cv::Mat binary;
    cv::threshold(src, binary, 120, 255, cv::THRESH_BINARY_INV);//THRESH_BINARY_INV

    cv::Mat k55 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, k55, cv::Point(-1, -1), 1);


    //cv::namedWindow("binary",cv::WINDOW_NORMAL);
    //cv::imshow("binary", binary);


    std::vector<std::vector<cv::Point> > defectContours;
    std::vector<cv::Vec4i> hierarchyDefect;
    cv::findContours(binary, defectContours, hierarchyDefect, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Mat contours = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));

    cv::Mat drawCenterPos = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawRect = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawHull = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawRotateRect = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawMinEnclosingCircle = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawMinEnclosingTriangle = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat drawEllipse = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));

    std::vector<std::vector<cv::Point>> PolyDPs;
    cv::Mat drawPolyDP = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));


    std::vector<std::vector<cv::Point> > hull(defectContours.size());
    //qDebug()<<"轮廓总数是 "<<defectContours.size();
    ksk_number = defectContours.size();
    if(cal_deep_shallow) {
        for(unsigned long i = 0; i < defectContours.size(); i++) {
            cv::drawContours(contours, defectContours, i, cv::Scalar(255,255,255), 1);
            //最大外接矩形 蓝色
            cv::Rect rect = cv::boundingRect(defectContours[i]);
            cv::rectangle(drawRect, rect, cv::Scalar(255,0,0));

             cv::Mat roi_area(src, rect);
             cv::Scalar tempVal = cv::mean(roi_area);
             double max, min;
             cv::Point min_loc, max_loc;
             cv::minMaxLoc(roi_area, &min, &max, &min_loc, &max_loc);
             cv::Scalar srcAvg = cv::mean(src).val[0];
             qDebug()<<"ROI的均值是：" << srcAvg.val[0]<<"点 "<< i<<" 的均值是：" << tempVal.val[0]<<"，最值是："<< min<<"，"<<max;
             if((tempVal.val[0]/srcAvg.val[0] <= 0.55f)) {
                 mDeepNumber ++;
             } else if((min/max >= 0.8f) && (tempVal.val[0]/srcAvg.val[0] <= 0.65f)) {
                 mDeepNumber ++;
             } else {
                 mShallowNumber++;
             }
        }
    }
#if 0
    for(unsigned long i = 0; i < defectContours.size(); i++) {
       //轮廓 白色
       cv::drawContours(contours, defectContours, i, cv::Scalar(255,255,255), 1);
//       putText(contours, std::to_string(i), defectContours[i][0],FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,1);

//       //求面积
//       double fArea = cv::contourArea(defectContours[i]);
//       qDebug()<<"轮廓序号["<<i<<"]"<<QStringLiteral("面积")<<fArea;

//       //求周长
//       double fLength = cv::arcLength(defectContours[i],true);
//       qDebug()<<"轮廓序号["<<i<<"]"<<QStringLiteral("周长")<<fLength;

//       //矩求质心 红色
//       cv::Moments mu = cv::moments(defectContours[i]);
//       cv::Point2d centerPos = cv::Point2d( mu.m10/mu.m00 , mu.m01/mu.m00);
//       cv::circle(drawCenterPos, centerPos, 10, cv::Scalar(0, 0, 255));

       //最大外接矩形 蓝色
       cv::Rect rect = cv::boundingRect(defectContours[i]);
       cv::rectangle(drawRect, rect, cv::Scalar(255,0,0));

        cv::Mat roi_area(src, rect);
        cv::Scalar tempVal = cv::mean(roi_area);
        double max, min;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(roi_area, &min, &max, &min_loc, &max_loc);
        cv::Scalar srcAvg = cv::mean(src).val[0];
        qDebug()<<"ROI的均值是：" << srcAvg.val[0]<<"点 "<< i<<" 的均值是：" << tempVal.val[0]<<"，最值是："<< min<<"，"<<max;
        if((tempVal.val[0]/srcAvg.val[0] <= 0.55f)) {
            mDeepNumber ++;
        } else if((min/max >= 0.8f) && (tempVal.val[0]/srcAvg.val[0] <= 0.65f)) {
            mDeepNumber ++;
        } else {
            mShallowNumber++;
        }

//       //凸包 黄色
//       cv::convexHull(cv::Mat(defectContours[i]), hull[i], false);
//       cv::drawContours(drawHull, hull, i, cv::Scalar(0,255,255), 1);

//       //最小外接矩形 青色
//       cv::RotatedRect rotateRect = cv::minAreaRect(defectContours[i]);
//       vector<cv::Point2f> boxPts(4);
//       rotateRect.points(boxPts.data());

//       for(int j = 0; j < 4; j++) {
//           cv::line(drawRotateRect, boxPts[j], boxPts[(j + 1) % 4], cv::Scalar(128, 128, 0), 1, 8);  //绘制最小外接矩形每条边
//       }



//       //最小外接圆 棕褐色
//       cv::Point2f center;
//       float fRadius = 0;
//       cv::minEnclosingCircle(defectContours[i], center, fRadius);
//       cv::circle(drawMinEnclosingCircle, center, fRadius, cv::Scalar(140,180,210),1);

//       //最小外界三角形 粉色
//       std::vector<cv::Point2f> points;
//       cv::minEnclosingTriangle(defectContours[i], points);
//       cv::line(drawMinEnclosingTriangle, points[0], points[1], cv::Scalar(193, 182, 255), 1, 8);  //绘制最小外接三角形每条边
//       cv::line(drawMinEnclosingTriangle, points[2], points[1], cv::Scalar(193, 182, 255), 1, 8);  //绘制最小外接三角形每条边
//       cv::line(drawMinEnclosingTriangle, points[2], points[0], cv::Scalar(193, 182, 255), 1, 8);  //绘制最小外接三角形每条边


//       //椭圆 紫罗兰色
//       cv::RotatedRect ellipse =  cv::fitEllipse(defectContours[i]);
//       cv::ellipse(drawEllipse, ellipse, cv::Scalar(226,43,138));

//       //多边形逼近 洋红色
//       std::vector<cv::Point> PolyDP;
//       //cv::Mat PolyDP ;
//       cv::approxPolyDP(defectContours[i],PolyDP, 0.5, true);
//       PolyDPs.push_back(PolyDP);
//       //cv::drawContours(drawPolyDP, PolyDP, 0, cv::Scalar(23,56,234));
//       //drawPolyDP += cv::Mat(PolyDP);
//    }

//    //画多边形逼近
//    for(unsigned long j = 0; j < PolyDPs.size(); j++) {
//       cv::drawContours(drawPolyDP, PolyDPs, j, cv::Scalar(255,0,255),1);
//    }
}

//    drawCenterPos += contours;
//    drawRect += contours;
//    drawHull += contours;
//    drawRotateRect += contours;
//    drawMinEnclosingCircle += contours;
//    drawMinEnclosingTriangle += contours;
//    drawEllipse += contours;


//    cv::namedWindow("contours",cv::WINDOW_NORMAL);
//    cv::imshow("contours", contours);

//    cv::namedWindow("drawRect",cv::WINDOW_NORMAL);
//    cv::imshow("drawRect", drawRect);

//    cv::namedWindow("drawHull",cv::WINDOW_NORMAL);
//    cv::imshow("drawHull", drawHull);

//    cv::namedWindow("drawRotateRect",cv::WINDOW_NORMAL);
//    cv::imshow("drawRotateRect", drawRotateRect);

//    cv::namedWindow("drawCenterPos",cv::WINDOW_NORMAL);
//    cv::imshow("drawCenterPos", drawCenterPos);

//    cv::namedWindow("drawMinEnclosingCircle",cv::WINDOW_NORMAL);
//    cv::imshow("drawMinEnclosingCircle", drawMinEnclosingCircle);

//    cv::namedWindow("drawMinEnclosingTriangle",cv::WINDOW_NORMAL);
//    cv::imshow("drawMinEnclosingTriangle", drawMinEnclosingTriangle);

//    cv::namedWindow("drawEllipse",cv::WINDOW_NORMAL);
//    cv::imshow("drawEllipse", drawEllipse);

//    cv::namedWindow("drawPolyDP",cv::WINDOW_NORMAL);
//    cv::imshow("drawPolyDP", drawPolyDP);
#endif

    binary.release();
    k55.release();
    contours.release();
    drawCenterPos.release();
    drawRect.release();
    drawHull.release();
    drawRotateRect.release();
    drawMinEnclosingCircle.release();
    drawMinEnclosingTriangle.release();
    drawEllipse.release();
    qDebug()<<"深坑数量是"<<mDeepNumber<<"，浅坑数量是"<<mShallowNumber;
    return ksk_number;
}



void MainWindow::on_btn_show_capture_clicked()
{
    if(mPicFilepath.isEmpty()) {
        QMessageBox::warning(this,tr("Error"),tr("请选择正确的文件。"));
        return;
    }
    if(mRoiX < 0 || mRoiY < 0 || abs(mRoiW) <= 50 || abs(mRoiH) <= 50) {
        QMessageBox::warning(this,tr("Error"),tr("请选择ROI区域。"));
        return;
    }
    if(mRoiW < 0) {
        mRoiW = abs(mRoiW);
        mRoiX = mRoiX - mRoiW;
    }
    if(mRoiH < 0) {
        mRoiH = abs(mRoiH);
        mRoiY = mRoiY - mRoiH;
    }
    mRoiImage = mSrcImage(Rect(mRoiX, mRoiY, mRoiW, mRoiH)).clone();
    //cvtColor(img, img, COLOR_RGB2BGR);
#if 1
    cv::namedWindow("ROI",cv::WINDOW_NORMAL);
    cv::imshow("ROI", mRoiImage);
    mRoiImage.release();
#else
    cvtColor(mRoiImage, mRoiImage, COLOR_BGR2RGB);
    QImage img = QImage((const unsigned char*)(mRoiImage.data),
                        mRoiImage.cols, mRoiImage.rows,
                        mRoiImage.cols*mRoiImage.channels(),
                        QImage::Format_RGB888).scaled(mRoiImage.cols,mRoiImage.rows);

    labelROI = new QLabel(this, Qt::Dialog |Qt::WindowCloseButtonHint); /* 去掉?按钮 */
    labelROI->setWindowTitle("ROI");
    labelROI->installEventFilter(this);

    labelROI->resize(QSize(img.width(), img.height()));
    labelROI->setPixmap(QPixmap::fromImage(img));
    mRoiImage.release();
    labelROI->show();

#endif

    //ui->text_ksk_num->setText(QString::number(analysis(img)));

}

Mat doRuihua(Mat& src) {
    Mat dstImage;
#if 0
    Mat kern = (Mat_<char>(3,3) <<  0, -1 ,0,
                                   -1, 5, -1,
                                    0, -1, 0);
    Mat dstImage;
    filter2D(mRoiImage,dstImage,mRoiImage.depth(),kern);
    namedWindow("dstImage",WINDOW_AUTOSIZE);
    imshow("dstImage",dstImage);
#endif
#if 1 //高斯锐化
    Mat blur_img;

    double weight = -0.5;
    //GaussianBlur(mRoiImage, blur_img, Size(5, 5), 0, 0);   // 对灰度图效果较好

    GaussianBlur(src, blur_img, Size(0, 0), 25);  // 对彩色图效果较好

    addWeighted(src, 1-(weight), blur_img, weight, 0, dstImage);

    //namedWindow("dstImage",WINDOW_AUTOSIZE);
    //imshow("dstImage",dstImage);
#endif
#if 0 //拉普拉斯锐化
    Mat dstImage;
    double weight = 1.0;
    Mat& m = mRoiImage; m.copyTo(dstImage); int w1 = (m.cols - 1), h1 = (m.rows - 1);
    int la;
    int row_size = dstImage.step, pixel_size = dstImage.elemSize(); uint8_t* buf, * row;
    row = (uint8_t*)dstImage.data + row_size; buf = row + pixel_size;
    int row_size0 = m.step, pixel_size0 = m.elemSize(); uint8_t* buf0, * buf01, * buf02, * row0;
    row0 = (uint8_t*)m.data + row_size0; buf0 = row0 + pixel_size0; buf01 = buf0 - row_size0; buf02 = buf0 + row_size0;
    for(int i = 1; i < h1; i++) {
        for(int j = 1; j < w1; j++) {
            //la = 4 * m.at(i, j) - m.at(i + 1, j) - m.at(i - 1, j) - m.at(i, j + 1) - m.at(i, j - 1); nowImg.at(i, j) = saturate_cast(nowImg.at(i, j) + la);
            la = 4 * (*buf0) - *buf02 - *buf01 - *(buf0 + pixel_size0) - *(buf0 - pixel_size0); *buf= saturate_cast<uchar>(*buf0 + la*weight);
            buf += pixel_size; buf0 += pixel_size0; buf01 += pixel_size0; buf02 += pixel_size0;
        }
        row += row_size; buf = row + pixel_size;
        row0 += row_size0; buf0 = row0 + pixel_size0; buf01 = buf0 - row_size0; buf02 = buf0 + row_size0;
    }
    namedWindow("dstImage",WINDOW_AUTOSIZE);
    imshow("dstImage",dstImage);
#endif

    return dstImage;
}

void MainWindow::on_btn_cal_ksk_clicked()
{
    if(mRoiX < 0 || mRoiY < 0 || abs(mRoiW) <= 50 || abs(mRoiH) <= 50) {
        QMessageBox::warning(this,tr("Error"),tr("请选择ROI区域。"));
        return;
    }
    showLoadingDialog();
    RunnableKSKCalculator* ptr = new RunnableKSKCalculator();
    QThreadPool::globalInstance()->start(ptr);
    //std::thread tProcessor = std::thread(&MainWindow::KSKCalculator, this);
    //tProcessor.detach();
}


void MainWindow::on_btn_demarcate_clicked()
{
    //qDebug()<<__func__;
    if(mPicFilepath.isEmpty()) {
        QMessageBox::warning(this,tr("Error"),tr("请选择正确的文件。"));
        return;
    }
    mPreviewLabel->enterDemarcate();


}


void MainWindow::on_btn_10x10_cal_ksk_clicked()
{
    if((mPreviewLabel->getEndPointDemarcate().x() - mPreviewLabel->getStartPointDemarcate().x()) <= 50 || (mPreviewLabel->getEndPointDemarcate().y() - mPreviewLabel->getStartPointDemarcate().y()) <= 50) {
        QMessageBox::warning(this,tr("Error"),tr("请先标定籽晶位置"));
        return;
    }
    showLoadingDialog();
    RunnableKSK10x10Calculator* ptr = new RunnableKSK10x10Calculator();
    QThreadPool::globalInstance()->start(ptr);
}

void MainWindow::showLoadingDialog() {
    //实例化loading窗口
    ptrLoadingDialog = new waiting(this);
    ptrLoadingDialog->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    ptrLoadingDialog->setWindowModality(Qt::ApplicationModal);
    ptrLoadingDialog->move(880,450);
    ptrLoadingDialog->show();
    connect(ptrMainWindow, SIGNAL(send2Loading()), ptrLoadingDialog, SIGNAL(loading()));
}

void MainWindow::KSKCalculator() {
    if(mRoiX < 0 || mRoiY < 0 || abs(mRoiW) <= 50 || abs(mRoiH) <= 50) {
        QMessageBox::warning(this,tr("Error"),tr("请选择ROI区域。"));
    } else {
        int ksk_number = 0;
        if(mRoiImage.empty()) {
            Mat img = mSrcImage(Rect(mRoiX, mRoiY, mRoiW, mRoiH)).clone();
            img = doRuihua(img).clone();

            ksk_number = analysis(img, true);
            //ui->text_ksk_num->setText(QString::number(ksk_number));
            doAppendText(ui->text_ksk_num, QString::number(ksk_number));
        } else {
            mRoiImage = doRuihua(mRoiImage).clone();
            ksk_number = analysis(mRoiImage, true);
            //ui->text_ksk_num->setText(QString::number(ksk_number));
            doAppendText(ui->text_ksk_num, QString::number(ksk_number));
        }

        QString str_pixel_area = QString::number(mRoiW*mRoiH);
        str_pixel_area.push_back("pixel");
        QChar ch(0x00b2);
        str_pixel_area.push_back(ch);
        //ui->text_roi_pixel_area->setText(str_pixel_area);
        doAppendText(ui->text_roi_pixel_area, str_pixel_area);

        int demarcate_w;
        int demarcate_h;
        if(mPreviewLabel->getEndPointDemarcate().x() <= 0) {
            demarcate_w = mSrcImage.cols;
            demarcate_h = mSrcImage.rows;
        } else {
            demarcate_w = ((mPreviewLabel->getEndPointDemarcate().x() - mPreviewLabel->getStartPointDemarcate().x()) * mPreviewSrc.cols) / mPreviewLabel->size().width();
            demarcate_h = ((mPreviewLabel->getEndPointDemarcate().y() - mPreviewLabel->getStartPointDemarcate().y()) * mPreviewSrc.rows) / mPreviewLabel->size().height();
        }
        demarcate_w = (demarcate_w * mSrcImage.cols) / mPreviewSrc.cols;
        demarcate_h = (demarcate_h * mSrcImage.rows) / mPreviewSrc.rows;
        qDebug()<<demarcate_w<<" x "<<demarcate_h;

        double total_area = ui->text_totalarea->toPlainText().toDouble();
        qDebug()<<"total area is "<<total_area;
        double real_area = (((double)mRoiW*(double)mRoiH)*(total_area))/((double)demarcate_w*(double)demarcate_h);
        QString str_real_area = QString::number(real_area);
        str_real_area.push_back("mm");
        str_real_area.push_back(ch);
        //ui->text_roi_real_area->setText(str_real_area);
        doAppendText(ui->text_roi_real_area, str_real_area);


        double ksk_density = (((double)(ksk_number))/real_area)*100;
        QString str_ksk_density = QString::number(ksk_density, 'e', 4);
        str_ksk_density.push_back("个/cm");
        str_ksk_density.push_back(ch);
        //ui->text_ksk_density->setText(str_ksk_density);
        doAppendText(ui->text_ksk_density, str_ksk_density);
        doAppendText(ui->text_deep_num, QString(QString::number(mDeepNumber)));
        doAppendText(ui->text_shallow_num, QString(QString::number(mShallowNumber)));
    }
    //sleep(3);
    //ptrLoadingDialog->close();
    //QTimer::singleShot(1000, ptrMainWindow, SIGNAL(send2Loading()));
}

void MainWindow::KSK10x10Calculator() {
    if((mPreviewLabel->getEndPointDemarcate().x() - mPreviewLabel->getStartPointDemarcate().x()) <= 50 || (mPreviewLabel->getEndPointDemarcate().y() - mPreviewLabel->getStartPointDemarcate().y()) <= 50) {
        QMessageBox::warning(this,tr("Error"),tr("请先标定籽晶位置"));
        return;
    }

    Mat grid_img;
    //获取主屏分辨率
    QRect mRect;
    mRect = QGuiApplication::screens().at(0)->availableGeometry();
    qDebug()<<"display screen width:"<<mRect.width()<<"  height:"<<mRect.height();

    mDiv10x10 = mSrcImage.size().height/mRect.height()+1;
    mDiv10x10 = ((mSrcImage.size().width/mRect.width()+1) > mDiv10x10) ? (mSrcImage.size().width/mRect.width()+1) : mDiv10x10;
    qDebug()<<"10x10 div =>"<<mDiv10x10;
    cv::Size dsize = cv::Size(mSrcImage.size().width/mDiv10x10, mSrcImage.size().height/mDiv10x10);
    cv::Mat resize_img;
    cv::resize(mSrcImage, grid_img, dsize, 0, 0, cv::INTER_AREA);



    printf("dbg>> image rolxrow is %dx%d\n", grid_img.cols, grid_img.rows);
    int step = 10;
    int grid_img_width = grid_img.size().width;
    int grid_img_height = grid_img.size().height;
    printf("dbg>> image size is %dx%d\n", grid_img_width, grid_img_height);

    KskHeatmapPara m10x10HeatmapPara;
    memset(&m10x10HeatmapPara, 0, sizeof(KskHeatmapPara));
    m10x10HeatmapPara.points.clear();
    m10x10HeatmapPara.areas.clear();
    m10x10HeatmapPara.max_density = 0;
    memset(&m10x10HeatmapPara.color_table, 0, sizeof(m10x10HeatmapPara.color_table));
    Mat heatmap_final = grid_img.clone();
    Mat heatmap_img(grid_img.rows, grid_img.cols, CV_8UC4);
    qDebug() << "heatmap rolxrow is " << heatmap_img.cols << "x" << heatmap_img.rows;
    printf("dbg>> image size is %dx%d\n", heatmap_img.size().width, heatmap_img.size().height);
    InitAlpha2ColorTable(m10x10HeatmapPara.color_table);
    m10x10HeatmapPara.points.resize(100);
    m10x10HeatmapPara.areas.resize(100);

    int start_x = mPreviewLabel->getStartPointDemarcate().x()*grid_img_width/(mPreviewLabel->size().width());
    int start_y = mPreviewLabel->getStartPointDemarcate().y()*grid_img_height/(mPreviewLabel->size().height());
    int end_x = mPreviewLabel->getEndPointDemarcate().x()*grid_img_width/(mPreviewLabel->size().width());
    int end_y = mPreviewLabel->getEndPointDemarcate().y()*grid_img_height/(mPreviewLabel->size().height());

    cv::line(grid_img, Point(start_x, start_y), Point(start_x, end_y), cv::Scalar(0, 0, 255));
    cv::line(grid_img, Point(start_x, start_y), Point(end_x, start_y), cv::Scalar(0, 0, 255));
    cv::line(grid_img, Point(start_x, end_y), Point(end_x, end_y), cv::Scalar(0, 0, 255));
    cv::line(grid_img, Point(end_x, start_y), Point(end_x, end_y), cv::Scalar(0, 0, 255));
    for(int i = 0; i < (step-1); i++) {
        cv::line(grid_img, Point(start_x, start_y+(i+1)*(end_y-start_y)/step), Point(end_x, start_y+(i+1)*(end_y-start_y)/step), cv::Scalar(0, 0, 255));
        cv::line(grid_img, Point(start_x+(i+1)*(end_x-start_x)/step, start_y), Point(start_x+(i+1)*(end_x-start_x)/step, end_y), cv::Scalar(0, 0, 255));
    }
    Mat img;
    int ksk_number;
    int roi_x, roi_y, roi_w, roi_h;
    int demarcate_w;
    int demarcate_h;
    for(int i = 0; i < 10; i++) {
        for(int j = 0; j < 10; j++) {
            roi_x = ((start_x+(i)*(end_x-start_x)/step) * mSrcImage.cols) / grid_img.cols;
            roi_y = ((start_y+(j)*(end_y-start_y)/step) * mSrcImage.rows) / grid_img.rows;
            roi_w = (((end_x - start_x) * mSrcImage.cols) / grid_img.cols)/step;
            roi_h = (((end_y - start_y) * mSrcImage.rows) / grid_img.rows)/step;
            //qDebug()<<"roi_h"<<roi_x;
            //qDebug()<<"roi_w"<<roi_y;
            //qDebug()<<"roi_y"<<roi_w;
            //qDebug()<<"roi_x"<<roi_h;


            img = mSrcImage(Rect(roi_x, roi_y, roi_w, roi_h)).clone();
            img = doRuihua(img).clone();
            ksk_number = analysis(img);

            demarcate_w = ((end_x - start_x) * mSrcImage.cols) / grid_img.cols;
            demarcate_h = ((end_y - start_y) * mSrcImage.rows) / grid_img.rows;
            //qDebug()<<"demarcate_w"<<demarcate_w;
            //qDebug()<<"demarcate_h"<<demarcate_h;
            double total_area = ui->text_totalarea->toPlainText().toDouble();
            //qDebug()<<"total area is "<<total_area;
            double real_area = (((double)roi_w*(double)roi_h)*(total_area))/((double)demarcate_w*(double)demarcate_h);

            double ksk_density = (((double)(ksk_number))/real_area)*100;
            //qDebug()<<real_area;
            //qDebug()<<ksk_density;

            cv::putText(grid_img, QString::number(ksk_density, 'e', 4).toStdString().data(), Point(start_x+(i)*(end_x-start_x)/step, start_y+(j+1)*(end_y-start_y)/step - (i%2)*((end_y-start_y)/step/2) - 5), FONT_HERSHEY_SIMPLEX, 0.6f, Scalar(0, 255, 0), 1, 1);

            m10x10HeatmapPara.reach = (((end_x - start_x)/step)+((end_y - start_y)/step))/2+20;
            m10x10HeatmapPara.points[i*10+j].x = start_x+(i)*(end_x-start_x)/step + ((end_x - start_x)/step)/2;
            m10x10HeatmapPara.points[i*10+j].y = start_y+(j)*(end_y-start_y)/step + ((end_y - start_y)/step)/2;
            m10x10HeatmapPara.points[i*10+j].denskty = ksk_density;
            m10x10HeatmapPara.areas[i*10+j].left = (std::max)(start_x+(i)*(end_x-start_x)/step - m10x10HeatmapPara.reach, 0);
            m10x10HeatmapPara.areas[i*10+j].top = (std::max)(start_y+(j)*(end_y-start_y)/step - m10x10HeatmapPara.reach, 0);
            m10x10HeatmapPara.areas[i*10+j].right = (std::min)(start_x+(i)*(end_x-start_x)/step + ((end_x - start_x)/step) + m10x10HeatmapPara.reach, heatmap_img.size().width - 1);
            m10x10HeatmapPara.areas[i*10+j].bottom = (std::min)(start_y+(j)*(end_y-start_y)/step + ((end_y - start_y)/step) + m10x10HeatmapPara.reach, heatmap_img.size().height - 1);
            if(m10x10HeatmapPara.max_density < ksk_density) {
                m10x10HeatmapPara.max_density = ksk_density;
            }

        }
    }
    img.release();
    qDebug() << "max density is " << m10x10HeatmapPara.max_density;
    qDebug() << "heatmap reach is " << m10x10HeatmapPara.reach;
    //for(int i = 0; i < 100; i++) {
    //    m10x10HeatmapPara.points[i].weight = (int)((m10x10HeatmapPara.points[i].denskty/m10x10HeatmapPara.max_density)*100);
    //}
    int nBand = 4;
    uchar *data = heatmap_img.data;
    for (size_t i = 0; i < m10x10HeatmapPara.points.size(); i++) {
        //m10x10HeatmapPara.reach = m10x10HeatmapPara.reach*4;
        //权值因子
        float ratio = (float)(m10x10HeatmapPara.points[i].denskty/m10x10HeatmapPara.max_density);

        //遍历热力点范围
        for (int hi = m10x10HeatmapPara.areas[i].top; hi <= m10x10HeatmapPara.areas[i].bottom; hi++) {
          for (int wi = m10x10HeatmapPara.areas[i].left; wi <= m10x10HeatmapPara.areas[i].right; wi++) {
            //判断是否在热力圈范围
            float length =
                sqrt((float)(wi - m10x10HeatmapPara.points[i].x) * (wi - m10x10HeatmapPara.points[i].x) +
                     (hi - m10x10HeatmapPara.points[i].y) * (hi - m10x10HeatmapPara.points[i].y));
            if (length <= m10x10HeatmapPara.reach) {
              float alpha = ((m10x10HeatmapPara.reach - length) / m10x10HeatmapPara.reach) * ratio;

              //计算Alpha
              size_t m = (size_t)heatmap_img.cols * nBand * hi + wi * nBand;
              float newAlpha = data[m + 3] / 255.0f + alpha;
              newAlpha = std::min(std::max(newAlpha * 255, 0.0f), 255.0f);
              data[m + 3] = (uchar)(newAlpha);

              //颜色映射
              for (int bi = 0; bi < 3; bi++) {
                data[m + bi] = m10x10HeatmapPara.color_table[data[m + 3]][bi];
              }
            }
          }
        }
      }
    //imshow("heatmap", heatmap_img);
    int ruler_width = heatmap_final.size().width;

    cv::Mat ruler_mark(25, ruler_width, CV_8UC3);
    ruler_mark.setTo(0);
    for(int i = 10; i >= 1; i--) {
        String str = QString::number(m10x10HeatmapPara.max_density*((float)i/(10.0f)), 'e', 4).toStdString().data();
        cv::putText(ruler_mark, str, Point((ruler_width/10)*(10-i), 20), FONT_HERSHEY_SIMPLEX, 0.4f, Scalar(255, 255, 255), 1, 1);
    }

    cv::Mat heatmap_ruler(25, ruler_width, CV_8UC4);
    heatmap_ruler.setTo(0);
    uchar *heatmap_ruler_data = heatmap_ruler.data;
    for (int hi = 0; hi <= 24; hi++) {
        for (int wi = 0; wi <= ruler_width; wi++) {
            //判断是否在热力圈范围
            float length =(float)(wi - 0);
            if (length <= (float)ruler_width) {
                float alpha = (((float)ruler_width - length) / (float)ruler_width) * 1;

                //计算Alpha
                size_t m = ruler_width * nBand * hi + wi * nBand;
                float newAlpha = heatmap_ruler_data[m + 3] / 255.0f + alpha;
                newAlpha = std::min(std::max(newAlpha * 255, 0.0f), 255.0f);
                heatmap_ruler_data[m + 3] = (uchar)(newAlpha);

                //颜色映射
                for (int bi = 0; bi < 3; bi++) {
                    heatmap_ruler_data[m + bi] = m10x10HeatmapPara.color_table[heatmap_ruler_data[m + 3]][bi];
                }
            }
        }
    }
    printf("dbg>> image size is %dx%d\n", heatmap_ruler.size().width, heatmap_ruler.size().height);
    //cv::imshow("ruler", heatmap_ruler);


#if 0
    for(int i = grid_img_width/step; i < grid_img_height; i += grid_img_width/step) {
        cv::line(grid_img, Point(0, i), Point(grid_img_width, i), cv::Scalar(0, 0, 255));

    }
    for(int i = grid_img_height/step; i < grid_img_width; i += grid_img_height/step) {

        cv::line(grid_img, Point(i, 0), Point(i, grid_img_height), cv::Scalar(0, 0, 255));
        //cv::putText(grid_img, "calculating", Point(i, 0), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 1, 1);
    }
#endif
#if 0
    cv::imshow("resize image", grid_img);
#else
    //cvtColor(heatmap_ruler, heatmap_ruler, COLOR_RGBA2RGB);
    //Mat imageROI;
    //imageROI = heatmap_final(Range(heatmap_final.rows-heatmap_ruler.rows, heatmap_final.rows), Range(heatmap_final.cols-heatmap_ruler.cols, heatmap_final.cols));
    //heatmap_ruler.copyTo(imageROI);
    cvtColor(heatmap_final, heatmap_final, COLOR_RGB2RGBA);
    cv::addWeighted(heatmap_img, 0.3, heatmap_final, 1.0, 0, heatmap_final);
    cvtColor(heatmap_ruler, heatmap_ruler, COLOR_RGBA2RGB);
    cv::vconcat(heatmap_ruler, ruler_mark, heatmap_ruler);
    cvtColor(heatmap_final, heatmap_final, COLOR_RGBA2RGB);
    cv::Mat heatmap_output;
    cv::vconcat(heatmap_ruler, heatmap_final, heatmap_output);
    QImage qimage_heatmap = QImage((const unsigned char*)(heatmap_output.data),
                        heatmap_output.cols, heatmap_output.rows,
                        heatmap_output.cols*heatmap_output.channels(),
                        QImage::Format_RGB888).scaled(heatmap_output.cols,heatmap_output.rows);
    doShowImage(qimage_heatmap.copy(qimage_heatmap.rect()), QString("10x10 heatmap"));

//    label10x10Heatmap = new QLabel(this, Qt::Dialog |Qt::WindowCloseButtonHint); /* 去掉?按钮 */
//    label10x10Heatmap->setWindowTitle("10x10 Heatmap");
//    label10x10Heatmap->installEventFilter(this);
//
//    label10x10Heatmap->resize(QSize(qimage_heatmap.width(), qimage_heatmap.height()));
//    label10x10Heatmap->setPixmap(QPixmap::fromImage(qimage_heatmap.rgbSwapped()));
//
//    //heatmap_img.release();
//    //imshow("mixed", grid_img);
    QImage qimage = QImage((const unsigned char*)(grid_img.data),
                        grid_img.cols, grid_img.rows,
                        grid_img.cols*grid_img.channels(),
                        QImage::Format_RGB888).scaled(grid_img.cols,grid_img.rows);

    doShowImage(qimage.copy(qimage.rect()), QString("10x10 calculation"));
//    label10x10 = new QLabel(this, Qt::Dialog |Qt::WindowCloseButtonHint); /* 去掉?按钮 */
//    label10x10->setWindowTitle("10x10 calculation");
//    label10x10->installEventFilter(this);
//
//    label10x10->resize(QSize(qimage.width(), qimage.height()));
//    label10x10->setPixmap(QPixmap::fromImage(qimage.rgbSwapped()));
    grid_img.release();
    heatmap_final.release();
    heatmap_img.release();
    heatmap_ruler.release();
    ruler_mark.release();
//    label10x10->show();
//    label10x10Heatmap->show();
#endif
}

