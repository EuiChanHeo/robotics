#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <iostream>

QTimer DataTimer;

extern SHM sharedMemory;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    GraphInitialize();

    graphTimer = new QTimer();
    connect(graphTimer, SIGNAL(timeout()), this, SLOT(GraphUpdate()));
    graphTimer->start(20);
    graphOffset = 2.5;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::GraphInitialize()
{
    QPen myPen, dotPen, filterPen;
    myPen.setWidthF(1);
    filterPen.setStyle(Qt::DotLine);
    filterPen.setWidth(1);
    dotPen.setStyle(Qt::DotLine);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    dotPen.setColor(Qt::gray);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    ui->QCP_ForceTrajectory->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_Theta1_Trajectory->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_Theta2Trajectory->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    myPen.setWidthF(1.5);
    myPen.setColor(Qt::blue);
    
    ui->QCP_ForceTrajectory->legend->setFont(QFont("Helvetica", 9));
    ui->QCP_ForceTrajectory->addGraph();
    ui->QCP_ForceTrajectory->graph(0)->setPen(myPen);
    ui->QCP_ForceTrajectory->graph(0)->setName("Force Trajectory");
    ui->QCP_ForceTrajectory->xAxis->setLabel("time (s)");
    ui->QCP_ForceTrajectory->yAxis->setLabel("Force_z");
    ui->QCP_ForceTrajectory->xAxis->setTicker(timeTicker);
    ui->QCP_ForceTrajectory->yAxis->setRange(0, 70);

    myPen.setWidthF(1.5);
    myPen.setColor(Qt::blue);

    ui->QCP_Theta1_Trajectory->legend->setFont(QFont("Helvetica", 9));
    ui->QCP_Theta1_Trajectory->addGraph();
    ui->QCP_Theta1_Trajectory->graph(0)->setPen(myPen);
    ui->QCP_Theta1_Trajectory->graph(0)->setName("Theta_01 Trajectory");
    ui->QCP_Theta1_Trajectory->xAxis->setLabel("time (s)");
    ui->QCP_Theta1_Trajectory->yAxis->setLabel("Theta_1");
    ui->QCP_Theta1_Trajectory->xAxis->setTicker(timeTicker);
    ui->QCP_Theta1_Trajectory->yAxis->setRange(0, 70);

    myPen.setWidthF(1.5);
    myPen.setColor(Qt::blue);

    ui->QCP_Theta2Trajectory->legend->setFont(QFont("Helvetica", 9));
    ui->QCP_Theta2Trajectory->addGraph();
    ui->QCP_Theta2Trajectory->graph(0)->setPen(myPen);
    ui->QCP_Theta2Trajectory->graph(0)->setName("Theta_02 Trajectory");
    ui->QCP_Theta2Trajectory->xAxis->setLabel("time (s)");
    ui->QCP_Theta2Trajectory->yAxis->setLabel("Theta_2");
    ui->QCP_Theta2Trajectory->xAxis->setTicker(timeTicker);
    ui->QCP_Theta2Trajectory->yAxis->setRange(0, 70);

}

void MainWindow::GraphUpdate()
{
    connect(ui->QCP_ForceTrajectory->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_ForceTrajectory->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->QCP_ForceTrajectory->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_ForceTrajectory->yAxis2, SLOT(setRange(QCPRange)));

    connect(ui->QCP_Theta1_Trajectory->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_Theta1_Trajectory->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->QCP_Theta1_Trajectory->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_Theta1_Trajectory->yAxis2, SLOT(setRange(QCPRange)));

    connect(ui->QCP_Theta2Trajectory->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_Theta2Trajectory->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->QCP_Theta2Trajectory->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->QCP_Theta2Trajectory->yAxis2, SLOT(setRange(QCPRange)));

    DataTimer.start(0);
    static QTime time(QTime::currentTime());
    double key = time.elapsed() / 1000.0;
    static double lastPointKey = 0;
    if (key - lastPointKey > 0.001)
    {
        ui->QCP_ForceTrajectory   -> graph(0)->addData(key, sharedMemory.Force);
//        ui->QCP_Theta1_Trajectory -> graph(0)->addData(key, sharedMemory.currentPosition[8]);
//        ui->QCP_Theta2Trajectory  -> graph(0)->addData(key, sharedMemory.currentPosition[9]);

        lastPointKey = key;
    }

    ui->QCP_ForceTrajectory->rescaleAxes();
    ui->QCP_ForceTrajectory->replot();

    ui->QCP_Theta1_Trajectory->rescaleAxes();
    ui->QCP_Theta1_Trajectory->replot();

    ui->QCP_Theta2Trajectory->rescaleAxes();
    ui->QCP_Theta2Trajectory->replot();
}


