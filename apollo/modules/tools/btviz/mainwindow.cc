#include "mainwindow.h"
#include "modules/tools/btviz/ui_mainwindow.h"

#include <QDebug>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <FlowScene.h>
#include <NodeStyle.h>
#include <ConnectionStyle.h>
#include <FlowViewStyle.h>

using QtNodes::NodeStyle;
using QtNodes::ConnectionStyle;
using QtNodes::FlowViewStyle;
using QtNodes::FlowScene;

void MainWindow::setStyle()
{
  NodeStyle::setNodeStyle(
  R"(
  {
    "NodeStyle": 
    {
      "NormalBoundaryColor": [255, 242, 204],
      "SelectedBoundaryColor": [255, 165, 0],
      "GradientColor0": [255, 242, 204],
      "GradientColor1": [255, 242, 204],
      "GradientColor2": [255, 242, 204],
      "GradientColor3": [255, 242, 204],
      "ShadowColor": [200, 200, 200],
      "FontColor" : "black",
      "FontColorFaded" : "gray",
      "ConnectionPointColor": [169, 169, 169],
      "FilledConnectionPointColor": "cyan",
      "ErrorColor": "red",
      "WarningColor": [255, 173, 31],

      "PenWidth": 1.0,
      "HoveredPenWidth": 1.5,

      "ConnectionPointDiameter": 8.0,

      "Opacity": 1.0
    }
  }
  )");

  // ConnectionStyle::setConnectionStyle(
  // R"(
  // {
  //   "ConnectionStyle": {
  //     "ConstructionColor": "gray",
  //     "NormalColor": "black",
  //     "SelectedColor": "gray",
  //     "SelectedHaloColor": "deepskyblue",
  //     "HoveredColor": "deepskyblue",
  //     "LineWidth": 3.0,
  //     "ConstructionLineWidth": 2.0,
  //     "PointDiameter": 40.0,
  //   }
  // }
  // )");

  FlowViewStyle::setStyle(
  R"(
  {  
    "FlowViewStyle": {
    "BackgroundColor": [255, 255, 255],
    "FineGridColor": [240, 240, 240],
    "CoarseGridColor": [220, 220, 220]
    }
  }
  )");
}

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setStyle();

    btree_canvas_ = std::make_unique<BTvizCanvas>(this);
    
    ui->frameLayout->addWidget(btree_canvas_.get());
    ui->frameLayout->setContentsMargins(0, 0, 0, 0);
    ui->frameLayout->setSpacing(0);

    QObject::connect(ui->actionSave, &QAction::triggered,
                     btree_canvas_->getScene(), &FlowScene::save);

    QObject::connect(ui->actionLoad, &QAction::triggered,
                     btree_canvas_->getScene(), &FlowScene::load);

    // Scene is generic FlowScene object, so will need to handle this
    // QObject::connect(ui->actionGenerateProtobuf, &QAction::triggered,
                    //  btree_canvas_->scene(), &BTvizBTreeFlowScene::saveProtobuf);

    QObject::connect(ui->actionFitView, &QAction::triggered,
                     this, &MainWindow::fitToScreen);
}

void MainWindow::fitToScreen() const
{
  btree_canvas_->fitToScreen();
}

MainWindow::~MainWindow()
{
    delete ui;
}