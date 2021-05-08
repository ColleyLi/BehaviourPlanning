#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void setStyle();

private:
    Ui::MainWindow *ui;

    std::unique_ptr<BTvizCanvas> btree_canvas_;

    void fitToScreen() const;
};
#endif // MAINWINDOW_H