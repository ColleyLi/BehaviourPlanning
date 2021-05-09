#pragma once

#include <memory>
#include <map>
#include <vector>

#include <QMainWindow>

#include "modules/tools/btviz/btviz_canvas.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = Q_NULLPTR);
    ~MainWindow();

private:
    void loadDefaultStyle();

private:
    Ui::MainWindow *ui;

    std::unique_ptr<BTVizCanvas> btviz_canvas_;
};