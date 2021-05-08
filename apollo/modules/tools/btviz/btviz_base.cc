#include "modules/tools/btviz/btviz_base.h"

QString nodeCategoryFromNodeType(const QString& type)
{
    QStringList parts = type.split(' ', QString::SkipEmptyParts);

    return parts[parts.size() - 1] + "s"; 
}

QString BTreeContextTypeToQString(BTreeContextType type)
{
    const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeContextType_descriptor();
    QString s = QString::fromStdString(descriptor->FindValueByNumber(type)->name());
    QStringList parts = s.toLower().split('_', QString::SkipEmptyParts);
    for (int i = 0; i < parts.size(); ++i)
    {
        parts[i].replace(0, 1, parts[i][0].toUpper());
    }
    return parts.join(" ");
}

BTreeContextType QStringToBTreeContextType(QString string)
{
  const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeContextType_descriptor();
  QString s = string.toUpper().split(' ', QString::SkipEmptyParts).join("_");
  return static_cast<apollo::planning_btree::BTreeContextType>(descriptor->FindValueByName(s.toStdString())->number());
}

QString BTreeStageTypeToQString(BTreeStageType type)
{
    const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeStageType_descriptor();
    QString s = QString::fromStdString(descriptor->FindValueByNumber(type)->name());
    QStringList parts = s.toLower().split('_', QString::SkipEmptyParts);
    for (int i = 0; i < parts.size(); ++i)
    {
        parts[i].replace(0, 1, parts[i][0].toUpper());
    }
    return parts.join(" ");
}

BTreeStageType QStringToBTreeStageType(QString string)
{
  const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeStageType_descriptor();
  QString s = string.toUpper().split(' ', QString::SkipEmptyParts).join("_");
  return static_cast<apollo::planning_btree::BTreeStageType>(descriptor->FindValueByName(s.toStdString())->number());
}

QString BTreeNodeTypeToQString(BTreeNodeType type)
{
    const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeNodeType_descriptor();
    QString s = QString::fromStdString(descriptor->FindValueByNumber(type)->name());
    QStringList parts = s.toLower().split('_', QString::SkipEmptyParts);
    for (int i = 0; i < parts.size(); ++i)
    {
        parts[i].replace(0, 1, parts[i][0].toUpper());
    }
    return parts.join(" ");
}

BTreeNodeType QStringToBTreeNodeType(QString string)
{
  const google::protobuf::EnumDescriptor* descriptor = apollo::planning_btree::BTreeNodeType_descriptor();
  QString s = string.toUpper().split(' ', QString::SkipEmptyParts).join("_");
  return static_cast<apollo::planning_btree::BTreeNodeType>(descriptor->FindValueByName(s.toStdString())->number());
}

std::vector<BTvizNode> getExistingContextNodes()
{
    std::vector<BTvizNode> nodes;

    BTvizNode root_node;
    root_node.name = CONTEXT_ROOT_TYPE;
    root_node.category = CONTEXT_ROOT_TYPE;
    root_node.type = CONTEXT_ROOT_TYPE; 
        
    nodes.push_back(root_node);
    
    for (int i = apollo::planning_btree::BTreeContextType_MIN; i <= apollo::planning_btree::BTreeContextType_MAX; ++i)
    {
        if (!apollo::planning_btree::BTreeContextType_IsValid(i))
        {
            continue;
        }

        QString node_type = BTreeContextTypeToQString(static_cast<BTreeContextType>(i));
        
        BTvizNode node;
        node.name = node_type;
        node.category = nodeCategoryFromNodeType(node_type);
        node.type = node_type; 
        
        nodes.push_back(node);
    }

    return nodes;
}


std::vector<BTvizNode> getExistingStageNodes()
{
    std::vector<BTvizNode> nodes;

    BTvizNode root_node;
    root_node.name = STAGE_ROOT_TYPE;
    root_node.category = STAGE_ROOT_TYPE;
    root_node.type = STAGE_ROOT_TYPE; 
        
    nodes.push_back(root_node);
    
    for (int i = apollo::planning_btree::BTreeStageType_MIN; i <= apollo::planning_btree::BTreeStageType_MAX; ++i)
    {
        if (!apollo::planning_btree::BTreeStageType_IsValid(i))
        {
            continue;
        }

        QString node_type = BTreeStageTypeToQString(static_cast<BTreeStageType>(i));
        
        BTvizNode node;
        node.name = node_type;
        node.category = nodeCategoryFromNodeType(node_type);
        node.type = node_type; 
        
        nodes.push_back(node);
    }

    return nodes;
}

std::vector<BTvizNode> getExistingBTreeNodes()
{
    std::vector<BTvizNode> nodes;

    BTvizNode root_node;
    root_node.name = BTREE_ROOT_TYPE;
    root_node.category = BTREE_ROOT_TYPE;
    root_node.type = BTREE_ROOT_TYPE; 
        
    nodes.push_back(root_node);
    
    for (int i = apollo::planning_btree::BTreeNodeType_MIN; i <= apollo::planning_btree::BTreeNodeType_MAX; ++i)
    {
        if (!apollo::planning_btree::BTreeNodeType_IsValid(i))
        {
            continue;
        }

        QString node_type = BTreeNodeTypeToQString(static_cast<BTreeNodeType>(i));
        
        BTvizNode node;
        node.name = node_type;
        node.category = nodeCategoryFromNodeType(node_type);
        node.type = node_type; 
        
        nodes.push_back(node);
    }

    return nodes;
}