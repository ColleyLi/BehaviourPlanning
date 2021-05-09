#include "mainwindow.h"
#include "modules/tools/btviz/ui_mainwindow.h"

#include <QDebug>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#include <NodeStyle.h>
#include <ConnectionStyle.h>
#include <FlowViewStyle.h>

using QtNodes::NodeStyle;
using QtNodes::ConnectionStyle;
using QtNodes::FlowViewStyle;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    loadDefaultStyle();

    btviz_canvas_ = std::make_unique<BTVizCanvas>(this);
    
    ui->frameLayout->addWidget(btviz_canvas_.get());
    ui->frameLayout->setContentsMargins(0, 0, 0, 0);
    ui->frameLayout->setSpacing(0);

    QObject::connect(ui->actionSave, &QAction::triggered,
                     btviz_canvas_.get(), &BTVizCanvas::saveBTPlan);

    QObject::connect(ui->actionLoad, &QAction::triggered,
                     btviz_canvas_.get(), &BTVizCanvas::loadBTPlan);

    QObject::connect(ui->actionGenerateProtobuf, &QAction::triggered,
                     btviz_canvas_.get(), &BTVizCanvas::saveBTPlanProtobuf);

    QObject::connect(ui->actionFitView, &QAction::triggered,
                     btviz_canvas_.get(), &BTVizCanvas::fitToScreen);
}

void MainWindow::loadDefaultStyle()
{
  // TODO: fix lazy Bazel. Bazel will not recompile resources until resource file changes
  QString style_file = ":btviz_style.json";
  QFile file(style_file);

  if (!file.open(QIODevice::ReadOnly))
  {
    qWarning() << "Couldn't open default style file " << style_file;

    return;
  }

  QJsonDocument json_style(QJsonDocument::fromJson(file.readAll()));
  QJsonObject top_object = json_style.object();

  FlowViewStyle::setStyle(QJsonDocument(top_object).toJson(QJsonDocument::Compact));
  NodeStyle::setNodeStyle(QJsonDocument(top_object).toJson(QJsonDocument::Compact));
  ConnectionStyle::setConnectionStyle(QJsonDocument(top_object).toJson(QJsonDocument::Compact));
}

MainWindow::~MainWindow()
{
    delete ui;
}