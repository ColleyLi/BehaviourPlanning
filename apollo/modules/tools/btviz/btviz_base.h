#ifndef BTVIZ_BASE_H
#define BTVIZ_BASE_H

#include <QString>
#include <QPointF>
#include <QSizeF>
#include <QDebug>

#include <Node.hpp>
#include <QStringStdHash.hpp>

#include <unordered_map>
#include <memory>
#include <vector>

#include "modules/planning/proto/b_tree_config.pb.h"

using QtNodes::Node;
using apollo::planning::BTreeNodeState;
using apollo::planning::BTreeNodeType;

struct BTNode
{
    QString id;
    QString name;
    QString type;
    QString category;
};

std::vector<BTNode> getExistingNodes();

struct BTvizNode
{
    BTvizNode():
        graphic_node(nullptr)
    {
    }

    BTNode bt_node;
    
    QString parent_id;
    std::vector<QString> children_ids;
    
    BTreeNodeState state;
    QPointF position;
    Node* graphic_node;
};

class BTvizTree
{
    typedef  std::unordered_map<QString, std::shared_ptr<BTvizNode>> BTvizNodes;

    public:
        BTvizTree(){}
        ~BTvizTree();

        BTvizNodes& nodes() {return nodes_;}

        std::shared_ptr<BTvizNode> getNode(QString id) {return nodes_[id];}
        std::shared_ptr<BTvizNode> getRootNode();

        void addNode(std::shared_ptr<BTvizNode> node);
        void clear();

    private:
        BTvizNodes nodes_;
        QString root_node_id_;
};

Q_DECLARE_METATYPE(BTvizTree);

static int getUid()
{
    static int uid = 0;
    return uid++;
}

#endif // BTVIZ_BASE_H