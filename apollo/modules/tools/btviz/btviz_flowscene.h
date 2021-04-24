#ifndef BTVIZ_FLOWSCENE_H
#define BTVIZ_FLOWSCENE_H

#include <FlowScene.hpp>
#include <DataModelRegistry.hpp>

#include "modules/tools/btviz/btviz_base.h"

class BTvizFlowScene : public QtNodes::FlowScene
{
    public:
        BTvizFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject * parent = Q_NULLPTR);

        bool isLocked() const {return locked_;}
        void lock(bool lock) {locked_ = lock;}

        QtNodes::Node& createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) override;

    // private:
        // void dragEnterEvent(QGraphicsSceneDragDropEvent* event) override;
        // void dragLeaveEvent(QGraphicsSceneDragDropEvent* event) override;
        // void dropEvent(QGraphicsSceneDragDropEvent* event) override;
        // void dragMoveEvent(QGraphicsSceneDragDropEvent* event) override;
        // void keyPressEvent( QKeyEvent* event) override;

    private:
        bool locked_;
};

#endif // EDITOR_FLOWSCENE_H