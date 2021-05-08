#ifndef BTVIZ_BTREE_FLOWSCENE_H
#define BTVIZ_BTREE_FLOWSCENE_H

#include <FlowScene.h>
#include <DataModelRegistry.h>

#include "modules/tools/btviz/btviz_base.h"

class BTvizBTreeFlowScene : public QtNodes::FlowScene
{
    public:
        BTvizBTreeFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject * parent = Q_NULLPTR);

        bool isLocked() const {return locked_;}
        void lock() {locked_ = true;}
        void unlock() {locked_ = false;}

        QtNodes::Node& createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) override;
        QtNodes::Node& createNodeAtPosition(const QString& node_type, const QString& this_scene_id, const QString& stage_scene_id, const QString& stage_name, const QPointF& scene_pos);    

        void saveProtobuf() const;

    private:    
        void generateProtobuf(const QString& file_name) const;

    private:
        bool locked_;
};

#endif // BTVIZ_BTREE_FLOWSCENE_H