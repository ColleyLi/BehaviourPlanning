#ifndef BTVIZ_CANVAS_H
#define BTVIZ_CANVAS_H

#include <QObject>
#include <QWidget>
#include <QLineEdit>

#include "modules/tools/btviz/btviz_base.h"
#include "modules/tools/btviz/btviz_flowscene.h"

#include <Node.hpp>
#include <NodeData.hpp>
#include <FlowScene.hpp>
#include <DataModelRegistry.hpp>
#include <FlowView.hpp>

class BTvizCanvas : public QObject
{
    Q_OBJECT
public:
    explicit BTvizCanvas(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                              QWidget *parent = nullptr);

private:
    BTvizFlowScene* flow_scene_;
    QtNodes::FlowView*  flow_view_;
    
    std::shared_ptr<QtNodes::DataModelRegistry> model_registry_;
    
    bool signal_was_blocked_;
};

#endif // BTVIZ_CANVAS_H