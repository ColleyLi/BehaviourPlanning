#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <memory>
#include <map>
#include <vector>

#include <DataModelRegistry.hpp>
#include "modules/tools/btviz/btviz_canvas.h"

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
    void setStyle();

private:
    Ui::MainWindow *ui;
    std::shared_ptr<QtNodes::DataModelRegistry> model_registry_;
    std::vector<BTNode> nodes_;
};
#endif // MAINWINDOW_H