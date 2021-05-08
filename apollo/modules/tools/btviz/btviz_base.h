#pragma once

#ifndef BTVIZ_BASE_H
#define BTVIZ_BASE_H

#include <QString>
#include <QDebug>

#include <unordered_map>
#include <memory>
#include <vector>

#include "modules/planning_btree/proto/btree_context_config.pb.h"
#include "modules/planning_btree/proto/btree_stage_config.pb.h"
#include "modules/planning_btree/proto/btree_config.pb.h"
#include "cyber/cyber.h"

using apollo::planning_btree::BTreeContextState;
using apollo::planning_btree::BTreeContextType;
using apollo::planning_btree::BTreeStageState;
using apollo::planning_btree::BTreeStageType;
using apollo::planning_btree::BTreeNodeState;
using apollo::planning_btree::BTreeNodeType;
using apollo::planning_btree::BTreeConfig;
using apollo::planning_btree::BTreeNodeDescription;
using apollo::cyber::common::SetProtoToASCIIFile;

const QString CONTEXT_ROOT_TYPE = "BTPlan";
const QString STAGE_ROOT_TYPE = "Context";
const QString BTREE_ROOT_TYPE = "Stage";

struct BTvizNode
{
    QString id;
    QString name;
    QString type;
    QString category;
};

QString nodeCategoryFromNodeType(const QString& type);

// TODO: refactor to make those methods general. Templates?

QString BTreeContextTypeToQString(BTreeContextType type);
BTreeContextType QStringToBTreeContextType(QString string);

QString BTreeStageTypeToQString(BTreeStageType type);
BTreeStageType QStringToBTreeStageType(QString string);

QString BTreeNodeTypeToQString(BTreeNodeType type);
BTreeNodeType QStringToBTreeNodeType(QString string);

std::vector<BTvizNode> getExistingContextNodes();
std::vector<BTvizNode> getExistingStageNodes();
std::vector<BTvizNode> getExistingBTreeNodes();

#endif // BTVIZ_BASE_H