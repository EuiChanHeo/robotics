#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "../../../include/Variables/SharedMemory.h"
#include "/home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.h"
#include "/home/percy/robot_ws/EE3100704/tools/include/robotController.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private:

    Ui::MainWindow *ui;
    QTimer          *graphTimer;
    double graphOffset;

private slots:

    void GraphInitialize();

    void GraphUpdate();

};
#endif // MAINWINDOW_H
