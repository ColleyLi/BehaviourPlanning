#ifndef BTVIZ_CONTEXT_FLOWSCENE_H
#define BTVIZ_CONTEXT_FLOWSCENE_H

#include <FlowScene.h>
#include <DataModelRegistry.h>

#include "modules/tools/btviz/btviz_base.h"

class BTvizContextFlowScene : public QtNodes::FlowScene
{
    public:
        BTvizContextFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject * parent = Q_NULLPTR);

        bool isLocked() const {return locked_;}
        void lock() {locked_ = true;}
        void unlock() {locked_ = false;}

        QtNodes::Node& createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) override;
        QtNodes::Node& createNodeAtPosition(const QString& node_type, const QString& this_scene_id, const QString& btplan_id, const QString& btplan_name, const QPointF& scene_pos);    
        
        void saveProtobuf() const;

    private:    
        void generateProtobuf(const QString& file_name) const;

    private:
        bool locked_;
};

#endif // BTVIZ_Context_FLOWSCENE_H