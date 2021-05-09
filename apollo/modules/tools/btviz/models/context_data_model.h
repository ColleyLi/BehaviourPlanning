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

class ContextDataModel : public NodeDataModel
{
  Q_OBJECT

  public:
    ContextDataModel(const BTVizNode& node);
    ~ContextDataModel() override;

    void Init() override;
 
  public:
    unsigned int nPorts(PortType portType) const override;

    bool hasDynamicPorts(PortType) const override;

    ConnectionPolicy portOutConnectionPolicy(PortIndex) const final;
    
    ConnectionPolicy portInConnectionPolicy(PortIndex) const final;

    NodeDataType dataType(PortType , PortIndex ) const final;

    void setInData(std::shared_ptr<NodeData>, int) final {}

    std::shared_ptr<NodeData> outData(PortIndex port) final;

    QWidget *embeddedWidget() final;
  
  public: 
    void setNodeName(const QString& name) override;

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
    std::vector<QString> contexts_;

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