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

#include <NodeStyle.hpp>
#include<NodeDataModel.hpp>

#include "modules/tools/btviz/btviz_base.h"

#include <iostream>

class DecimalData;

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;
using QtNodes::NodeStyle;

class BTreeDataModel : public NodeDataModel
{
  Q_OBJECT

  public:
    BTreeDataModel(const BTNode& node);
    ~BTreeDataModel() override;

    void Init();
 
  public:
    unsigned int nPorts(PortType portType) const override;

    ConnectionPolicy portOutConnectionPolicy(PortIndex) const final;

    NodeDataType dataType(PortType , PortIndex ) const final;

    void setInData(std::shared_ptr<NodeData>, int) final {}

    std::shared_ptr<NodeData> outData(PortIndex port) final;

    QString name() const final;
    
    QWidget *embeddedWidget() final;
  public: 
    void setNodeName(const QString& name);
    
    void setNodeId(const QString& id);
  
    QJsonObject save() const override;

    void restore(QJsonObject const &) override;

  public slots:
    void updateNodeSize();

  signals:
    void nodeNameChanged();

  private:
    BTNode node_;

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