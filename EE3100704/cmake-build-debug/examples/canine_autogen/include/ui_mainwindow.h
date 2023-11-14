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
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QFrame *frame;
    QLineEdit *angle_1;
    QLineEdit *angle_2;
    QLineEdit *angle_3;
    QLineEdit *angle_4;
    QLineEdit *angle_5;
    QLineEdit *angle_6;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QTextEdit *textEdit;
    QPushButton *input_button;
    QFrame *frame_2;
    QLineEdit *res_1;
    QLineEdit *res_2;
    QLineEdit *res_3;
    QLineEdit *res_4;
    QLineEdit *res_5;
    QLineEdit *res_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QTextEdit *textEdit_2;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(470, 382);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(10, 10, 220, 320));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        angle_1 = new QLineEdit(frame);
        angle_1->setObjectName(QString::fromUtf8("angle_1"));
        angle_1->setGeometry(QRect(90, 50, 120, 25));
        angle_2 = new QLineEdit(frame);
        angle_2->setObjectName(QString::fromUtf8("angle_2"));
        angle_2->setGeometry(QRect(90, 90, 120, 25));
        angle_3 = new QLineEdit(frame);
        angle_3->setObjectName(QString::fromUtf8("angle_3"));
        angle_3->setGeometry(QRect(90, 130, 120, 25));
        angle_4 = new QLineEdit(frame);
        angle_4->setObjectName(QString::fromUtf8("angle_4"));
        angle_4->setGeometry(QRect(90, 170, 120, 25));
        angle_5 = new QLineEdit(frame);
        angle_5->setObjectName(QString::fromUtf8("angle_5"));
        angle_5->setGeometry(QRect(90, 210, 120, 25));
        angle_6 = new QLineEdit(frame);
        angle_6->setObjectName(QString::fromUtf8("angle_6"));
        angle_6->setGeometry(QRect(90, 250, 120, 25));
        label = new QLabel(frame);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(15, 50, 71, 20));
        label_2 = new QLabel(frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(15, 90, 71, 20));
        label_3 = new QLabel(frame);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(15, 130, 71, 20));
        label_4 = new QLabel(frame);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(15, 170, 71, 20));
        label_5 = new QLabel(frame);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(15, 210, 71, 20));
        label_6 = new QLabel(frame);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(15, 250, 71, 20));
        textEdit = new QTextEdit(frame);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setGeometry(QRect(0, 0, 220, 30));
        input_button = new QPushButton(frame);
        input_button->setObjectName(QString::fromUtf8("input_button"));
        input_button->setGeometry(QRect(10, 285, 200, 25));
        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(240, 10, 220, 320));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        res_1 = new QLineEdit(frame_2);
        res_1->setObjectName(QString::fromUtf8("res_1"));
        res_1->setGeometry(QRect(90, 50, 120, 25));
        res_2 = new QLineEdit(frame_2);
        res_2->setObjectName(QString::fromUtf8("res_2"));
        res_2->setGeometry(QRect(90, 90, 120, 25));
        res_3 = new QLineEdit(frame_2);
        res_3->setObjectName(QString::fromUtf8("res_3"));
        res_3->setGeometry(QRect(90, 130, 120, 25));
        res_4 = new QLineEdit(frame_2);
        res_4->setObjectName(QString::fromUtf8("res_4"));
        res_4->setGeometry(QRect(90, 170, 120, 25));
        res_5 = new QLineEdit(frame_2);
        res_5->setObjectName(QString::fromUtf8("res_5"));
        res_5->setGeometry(QRect(90, 210, 120, 25));
        res_6 = new QLineEdit(frame_2);
        res_6->setObjectName(QString::fromUtf8("res_6"));
        res_6->setGeometry(QRect(90, 250, 120, 25));
        label_7 = new QLabel(frame_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(14, 170, 71, 20));
        label_8 = new QLabel(frame_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(15, 90, 71, 20));
        label_9 = new QLabel(frame_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(15, 130, 71, 20));
        label_10 = new QLabel(frame_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(15, 50, 71, 20));
        label_11 = new QLabel(frame_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(15, 210, 71, 20));
        label_12 = new QLabel(frame_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(15, 250, 71, 20));
        textEdit_2 = new QTextEdit(frame_2);
        textEdit_2->setObjectName(QString::fromUtf8("textEdit_2"));
        textEdit_2->setGeometry(QRect(0, 0, 220, 30));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 470, 22));
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
        label->setText(QApplication::translate("MainWindow", "Angle_1  : ", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Angle_2  : ", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Angle_3  : ", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Angle_4  : ", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Angle_5  : ", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Angle_6  : ", nullptr));
        textEdit->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Input</span></p></body></html>", nullptr));
        input_button->setText(QApplication::translate("MainWindow", "Input", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Angle_4  : ", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Angle_2  : ", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Angle_3  : ", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Angle_1  : ", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Angle_5  : ", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "Angle_6  : ", nullptr));
        textEdit_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Output</span></p></body></html>", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
