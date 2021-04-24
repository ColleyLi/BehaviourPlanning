#include "mainwindow.h"
#include "modules/tools/btviz/ui_mainwindow.h"

#include <QDebug>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <NodeData.hpp>
#include <FlowScene.hpp>
#include <FlowView.hpp>
#include <NodeStyle.hpp>
#include <ConnectionStyle.hpp>
#include <FlowViewStyle.hpp>
#include <TypeConverter.hpp>

#include "modules/tools/btviz/models/btree_data_model.h"

using QtNodes::DataModelRegistry;
using QtNodes::FlowScene;
using QtNodes::FlowView;
using QtNodes::NodeStyle;
using QtNodes::ConnectionStyle;
using QtNodes::FlowViewStyle;
using QtNodes::TypeConverter;
using QtNodes::TypeConverterId;

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

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setStyle();

    model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    auto registerNode = [this](const BTNode& node)
    {
        QtNodes::DataModelRegistry::RegistryItemCreator creator;
        creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
        {
            auto ptr = new BTreeDataModel(node);
            return std::unique_ptr<BTreeDataModel>(ptr);
        };
        model_registry_->registerModel(node.category, creator, node.type);
    };

    for (const auto& node: getExistingNodes())
    {
      qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerNode(node);
      nodes_.push_back(node);
    }

    auto scene = new BTvizFlowScene(model_registry_, this);
    scene->setLayout(QtNodes::PortLayout::Vertical);
    
    ui->frameLayout->addWidget(new FlowView(scene));
    ui->frameLayout->setContentsMargins(0, 0, 0, 0);
    ui->frameLayout->setSpacing(0);

    QObject::connect(ui->actionSave, &QAction::triggered,
                     scene, &FlowScene::save);

    QObject::connect(ui->actionLoad, &QAction::triggered,
                     scene, &FlowScene::load);
}

MainWindow::~MainWindow()
{
    delete ui;
}