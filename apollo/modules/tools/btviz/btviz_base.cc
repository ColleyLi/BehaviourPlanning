#include "modules/tools/btviz/btviz_base.h"

void BTvizTree::clear()
{
    nodes_.clear();
}

BTvizTree::~BTvizTree()
{
    clear();
}

std::shared_ptr<BTvizNode> BTvizTree::getRootNode()
{
    return nodes_[root_node_id_];
}

void BTvizTree::addNode(std::shared_ptr<BTvizNode> node)
{
    if(node->parent_id.length())
    {
        nodes_[node->parent_id]->children_ids.push_back(node->bt_node.id);
    }
    else
    {
        clear();
        root_node_id_ = node->bt_node.id;
    }

    nodes_[node->bt_node.id] = node;
}

QString nodeTypeToQString(BTreeNodeType type)
{
    const google::protobuf::EnumDescriptor* descriptor = apollo::planning::BTreeNodeType_descriptor();
    QString s = QString::fromStdString(descriptor->FindValueByNumber(type)->name());
    QStringList parts = s.toLower().split('_', QString::SkipEmptyParts);
    for (int i = 0; i < parts.size(); ++i)
    {
        parts[i].replace(0, 1, parts[i][0].toUpper());
    }
    return parts.join(" ");
}

BTreeNodeType QStringToNodeType(QString string)
{
  const google::protobuf::EnumDescriptor* descriptor = apollo::planning::BTreeNodeType_descriptor();
  QString s = string.toUpper().split(' ', QString::SkipEmptyParts).join("_");
  return static_cast<apollo::planning::BTreeNodeType>(descriptor->FindValueByName(s.toStdString())->number());
}

QString nodeCategoryFromNodeName(const QString& name)
{
    QStringList parts = name.split(' ', QString::SkipEmptyParts);

    return parts[parts.size() - 1] + "s"; 
}

std::vector<BTNode> getExistingNodes()
{
    std::vector<BTNode> nodes;

    BTNode root_node;
    root_node.name = "Root";
    root_node.category = "Root";
    root_node.type = "Root"; 
        
    nodes.push_back(root_node);
    
    for (int i = apollo::planning::BTreeNodeType_MIN; i <= apollo::planning::BTreeNodeType_MAX; ++i)
    {
        if (!apollo::planning::BTreeNodeType_IsValid(i))
        {
            continue;
        }

        apollo::planning::BTreeNodeType node_type = static_cast<apollo::planning::BTreeNodeType>(i);
        QString node_name = nodeTypeToQString(node_type);
        
        BTNode node;
        node.name = node_name;
        node.category = nodeCategoryFromNodeName(node_name);
        node.type = node_name; 
        
        nodes.push_back(node);
    }

    return nodes;
}