#include <QMimeData>
#include <QStandardItemModel>
#include <QVariant>
#include <QGraphicsSceneDragDropEvent>
#include <QKeyEvent>
#include <QCursor>
#include <QApplication>
#include <QGraphicsView>
#include <QtWidgets/QFileDialog>
#include <Node.h>

#include "modules/tools/btviz/flowscenes/btviz_stage_flowscene.h"

BTVizStageFlowScene::BTVizStageFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                                 QObject * parent): FlowScene(registry, parent)
{
}
