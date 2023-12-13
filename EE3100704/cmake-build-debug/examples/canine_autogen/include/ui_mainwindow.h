/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "/home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QCustomPlot *QCP_ForceTrajectory;
    QCustomPlot *QCP_HeadTrajectory;
    QCustomPlot *QCP_Theta1_Trajectory;
    QCustomPlot *QCP_Theta2Trajectory;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(961, 770);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        QCP_ForceTrajectory = new QCustomPlot(centralwidget);
        QCP_ForceTrajectory->setObjectName(QString::fromUtf8("QCP_ForceTrajectory"));
        QCP_ForceTrajectory->setGeometry(QRect(10, 10, 451, 351));
        QCP_HeadTrajectory = new QCustomPlot(centralwidget);
        QCP_HeadTrajectory->setObjectName(QString::fromUtf8("QCP_HeadTrajectory"));
        QCP_HeadTrajectory->setGeometry(QRect(10, 370, 451, 351));
        QCP_Theta1_Trajectory = new QCustomPlot(centralwidget);
        QCP_Theta1_Trajectory->setObjectName(QString::fromUtf8("QCP_Theta1_Trajectory"));
        QCP_Theta1_Trajectory->setGeometry(QRect(470, 10, 451, 351));
        QCP_Theta2Trajectory = new QCustomPlot(centralwidget);
        QCP_Theta2Trajectory->setObjectName(QString::fromUtf8("QCP_Theta2Trajectory"));
        QCP_Theta2Trajectory->setGeometry(QRect(470, 370, 451, 351));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 961, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
