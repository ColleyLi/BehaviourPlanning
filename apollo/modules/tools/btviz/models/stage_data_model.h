#pragma once

#include <QtCore/QObject>
#include <QtCore/QJsonObject>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QLineEdit>
#include <QSvgRenderer>
#include <QFile>
#include <QDebug>
#include <QJsonDocument>

#include <Node.h>
#include <NodeStyle.h>
#include <NodeDataModel.h>
#include <Connection.h>

#include "modules/tools/btviz/btviz_base.h"

#include <iostream>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::Node;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;
using QtNodes::NodeStyle;
using QtNodes::Connection;

class StageDataModel : public NodeDataModel
{
  Q_OBJECT

  public:
    StageDataModel(const BTvizNode& node);
    ~StageDataModel() override;

    void Init();
 
  public:

    QString caption() const override {return "";}

    bool captionVisible() const override { return false; }

    unsigned int nPorts(PortType portType) const override;

    bool hasDynamicPorts(PortType) const override;

    ConnectionPolicy portOutConnectionPolicy(PortIndex) const final;
    
    ConnectionPolicy portInConnectionPolicy(PortIndex) const final;

    NodeDataType dataType(PortType , PortIndex ) const final;

    void setInData(std::shared_ptr<NodeData>, int) final {}

    std::shared_ptr<NodeData> outData(PortIndex port) final;

    QString name() const final;

    QWidget *embeddedWidget() final;
  
  public:
    const QString& getContextSceneId();
    void setContextSceneId(const QString& scene_id);
    
    const QString& getContextName();
    void setContextName(const QString& name);
    
    const QString& getBTreeSceneId();
    void setBTreeSceneId(const QString& scene_id);

    void setNodeName(const QString& name);
    
    void setNodeId(const QString& id);

    const QString& getNodeId();
    
    const QString& getNodeName();
    
    const QString& getNodeType();
    
    const QString& getNodeCategory();

    QJsonObject save() const override;

    void restore(QJsonObject const &) override;

  public Q_SLOTS:
    void outputConnectionCreated(Connection const& c) override;
    
    void outputConnectionDeleted(Connection const& c) override;

  public slots:
    void updateNodeSize();

  signals:
    void nodeNameChanged();

  private:
    BTvizNode node_;

    QString context_scene_id_;
    QString context_name_;

    QString btree_scene_id_;
    
    std::vector<QString> in_list_;
    std::vector<QString> out_list_;

    QFrame*  main_widget_;
    QVBoxLayout* main_layout_;

    QLabel* caption_label_;
    QFrame* caption_logo_left_;
    QFrame* caption_logo_right_;

    QLineEdit* node_name_line_edit_;

    QSvgRenderer* icon_renderer_;

    QString style_icon_;
    QColor  style_color_;
};