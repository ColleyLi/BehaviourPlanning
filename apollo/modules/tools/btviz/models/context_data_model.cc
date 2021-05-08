#include "context_data_model.h"

#define MARGIN 10

ContextDataModel::ContextDataModel(const BTvizNode & node):
  node_(node)
{
  main_widget_ = new QFrame();
  node_name_line_edit_ = new QLineEdit(main_widget_);
  
  main_layout_ = new QVBoxLayout(main_widget_);
  main_widget_->setLayout(main_layout_);

  auto caption_layout = new QHBoxLayout();
  caption_label_ = new QLabel();
  caption_logo_left_  = new QFrame();
  caption_logo_right_ = new QFrame();
  caption_logo_left_->setFixedSize(QSize(0, 20));
  caption_logo_right_->setFixedSize(QSize(0, 20));
  caption_label_->setFixedHeight(20);

  QFont caption_font = caption_label_->font();
  caption_font.setPointSize(12);
  caption_label_->setFont(caption_font);

  caption_layout->addWidget(caption_label_, 0, Qt::AlignHCenter);

  main_layout_->addLayout(caption_layout);
  main_layout_->setMargin(0);
  main_layout_->setSpacing(2);
  
  node_name_line_edit_->setAlignment(Qt::AlignCenter);
  node_name_line_edit_->setText(node_.name);
  node_name_line_edit_->setFixedWidth(100);
  node_name_line_edit_->setStyleSheet("color: black;"
                                      "background-color: transparent;"
                                      "border: 0px;");
  main_layout_->addWidget(node_name_line_edit_);

  main_widget_->setAttribute(Qt::WA_NoSystemBackground);
  caption_layout->setSizeConstraint(QLayout::SizeConstraint::SetMaximumSize);
  main_layout_->setSizeConstraint(QLayout::SizeConstraint::SetMaximumSize);

  connect(node_name_line_edit_, &QLineEdit::editingFinished, this, [this]()
  {
    setNodeName(node_name_line_edit_->text());
  });
}

ContextDataModel::~ContextDataModel()
{

}

void ContextDataModel::Init()
{
  if(!style_icon_.isEmpty())
  {
    caption_logo_left_->setFixedWidth(20);
    caption_logo_right_->setFixedWidth(1);

    QFile file(style_icon_);
    if(!file.open(QIODevice::ReadOnly))
    {
      qDebug() << style_icon_ << " file open failed";
      file.close();
    }
    else 
    {
      QByteArray ba = file.readAll();
      // QByteArray new_color_fill = QString("fill:%1;").arg(_style_caption_color.name() ).toUtf8();
      // ba.replace("fill:#ffffff;", new_color_fill);
      icon_renderer_ = new QSvgRenderer(ba, this);
    }
  }

  caption_label_->setText(node_.type);
  caption_label_->setStyleSheet("font-weight: bold;");

  QPalette caption_palette = caption_label_->palette();
  caption_palette.setColor(caption_label_->backgroundRole(), Qt::transparent);
  // capt_palette.setColor(caption_label_->foregroundRole(), _style_caption_color);

  if (node_.category == CONTEXT_ROOT_TYPE)
  {
    setNodeStyle(NodeStyle(
    R"(
    {
        "NodeStyle": 
        {
        "NormalBoundaryColor": [110, 211, 255],
        "SelectedBoundaryColor": [255, 165, 0],
        "GradientColor0": [110, 211, 255],
        "GradientColor1": [110, 211, 255],
        "GradientColor2": [110, 211, 255],
        "GradientColor3": [110, 211, 255],
        "ShadowColor": [200, 200, 200],
        "FontColor" : "black",
        "FontColorFaded" : "gray",
        "ConnectionPointColor": [169, 169, 169],
        "FilledConnectionPointColor": "cyan",
        "ErrorColor": "red",
        "WarningColor": [255, 173, 31],

        "PenWidth": 1.0,
        "HoveredPenWidth": 1.5,

        "ConnectionPointDiameter": 8.0,

        "Opacity": 1.0
        }
    }
    )"));
  }
  else
  {
    setNodeStyle(NodeStyle(
    R"(
    {
        "NodeStyle": 
        {
        "NormalBoundaryColor": [255, 138, 222],
        "SelectedBoundaryColor": [255, 165, 0],
        "GradientColor0": [255, 138, 222],
        "GradientColor1": [255, 138, 222],
        "GradientColor2": [255, 138, 222],
        "GradientColor3": [255, 138, 222],
        "ShadowColor": [200, 200, 200],
        "FontColor" : "black",
        "FontColorFaded" : "gray",
        "ConnectionPointColor": [169, 169, 169],
        "FilledConnectionPointColor": "cyan",
        "ErrorColor": "red",
        "WarningColor": [255, 173, 31],

        "PenWidth": 1.0,
        "HoveredPenWidth": 1.5,

        "ConnectionPointDiameter": 8.0,

        "Opacity": 1.0
        }
    }
    )"));
  }

  caption_label_->setPalette(caption_palette);

  caption_logo_left_->adjustSize();
  caption_logo_right_->adjustSize();
  caption_label_->adjustSize();

  updateNodeSize();
}

unsigned int ContextDataModel::nPorts(PortType portType) const
{
  int result = 0;
  if(portType == QtNodes::PortType::Out)
  { 
    if (node_.category == CONTEXT_ROOT_TYPE)
    {
      result = contexts_.size() + 1;
    }
    else
    {
      result = 0;
    }
  }
  else if( portType == QtNodes::PortType::In)
  {
    if (node_.category == CONTEXT_ROOT_TYPE)
    {
      result = 0;
    }
    else
    {
      result = 1;
    }
  }

  return result;
}

void ContextDataModel::outputConnectionCreated(Connection const& c)
{
  int out_port_index = static_cast<int>(c.getPortIndex(QtNodes::PortType::Out));

  if ((out_port_index == static_cast<int>(contexts_.size())) && c.complete())
  {
    contexts_.push_back(c.getNode(QtNodes::PortType::In)->nodeDataModel()->name());
    if (node_.category == CONTEXT_ROOT_TYPE)
    {
      Q_EMIT portAdded(PortType::Out, static_cast<int>(contexts_.size()));
    }
  }
  else
  {
    contexts_[out_port_index] = c.getNode(QtNodes::PortType::In)->nodeDataModel()->name(); 
  }
}


void ContextDataModel::outputConnectionDeleted(Connection const& c)
{
  int out_port_index = static_cast<int>(c.getPortIndex(QtNodes::PortType::Out));
  
  if((out_port_index != -1) && (out_port_index != static_cast<int>(contexts_.size())) && (!c.complete()))
  {
    contexts_.erase(contexts_.begin() + out_port_index);
    if (node_.category == CONTEXT_ROOT_TYPE)
    {
      Q_EMIT portRemoved(PortType::Out, out_port_index);
    }

    for(int i = out_port_index; i < static_cast<int>(contexts_.size()); ++i)
    {
      Q_EMIT dataUpdated(i);
    }
  }
}

bool ContextDataModel::hasDynamicPorts(QtNodes::PortType portType) const
{
  return false;
}

NodeDataModel::ConnectionPolicy ContextDataModel::portOutConnectionPolicy(PortIndex) const
{
  return ConnectionPolicy::One;
}

NodeDataModel::ConnectionPolicy ContextDataModel::portInConnectionPolicy(PortIndex) const
{
  return ConnectionPolicy::One;
}

NodeDataType ContextDataModel:: dataType(PortType, PortIndex) const
{
  return NodeDataType();
}

std::shared_ptr<NodeData> ContextDataModel::outData(PortIndex)
{
  return nullptr;
}

QString ContextDataModel::name() const
{
  return node_.id;
}

QWidget* ContextDataModel::embeddedWidget()
{
  return main_widget_;
}

void ContextDataModel::setNodeName(const QString& name)
{
    node_.name = name;
    node_name_line_edit_->setText(name);

    updateNodeSize();
    emit nodeNameChanged();
}

void ContextDataModel::setNodeId(const QString& id)
{
  node_.id = id;
}

const QString& ContextDataModel::getNodeId()
{
  return node_.id;
}

const QString& ContextDataModel::getNodeName()
{
  return node_.name;
}

const QString& ContextDataModel::getNodeType()
{
  return node_.type;
}

const QString& ContextDataModel::getNodeCategory()
{
  return node_.category;
}

const QString& ContextDataModel::getStageSceneId()
{
  return stage_scene_id_;
}

void ContextDataModel::setStageSceneId(const QString& scene_id)
{
  stage_scene_id_ = scene_id;
}

void ContextDataModel::updateNodeSize()
{
    int caption_width = caption_label_->width();
    caption_width += caption_logo_left_->width() + caption_logo_right_->width();
    int line_edit_width = caption_width;

    if(node_name_line_edit_->isHidden() == false)
    {
        QFontMetrics fm = node_name_line_edit_->fontMetrics();
        const QString& txt = node_name_line_edit_->text();
        int text_width = fm.boundingRect(txt).width();
        line_edit_width = std::max(line_edit_width, text_width + MARGIN);
    }

    int field_colum_width = 50;

    field_colum_width = std::max(field_colum_width, line_edit_width);
    line_edit_width = std::max(line_edit_width, field_colum_width);
    node_name_line_edit_->setFixedWidth(line_edit_width);

    main_widget_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    main_widget_->adjustSize();

    emit embeddedWidgetSizeUpdated();
}

QJsonObject ContextDataModel::save() const
{
  QJsonObject node_json;

  node_json["name"] = node_.name;
  node_json["type"] = node_.type;
  node_json["id"] = node_.id;
  node_json["category"] = node_.category;

  // if ports are dynamics, write their value when saved.
  // when restored, model need to update the dynamic value.

//   if(hasDynamicPorts(PortType::In))
//   { 
//     int n_ports = static_cast<int>(nPorts(PortType::In));
//     if (n_ports > 0) n_ports--;
//     node_json["dynamic_inputs"]  = n_ports;
//   }

//   if(hasDynamicPorts(PortType::Out))
//   {
//     int n_ports = static_cast<int>(nPorts(PortType::Out));
//     if (n_ports > 0) n_ports--;
//     node_json["dynamic_outputs"] = n_ports;
//   }


  return node_json;
}


void ContextDataModel::restore(QJsonObject const &modelJson)
{
  node_.name = modelJson["name"].toString();
  node_.type = modelJson["type"].toString();
  node_.id = modelJson["id"].toString();
  node_.category = modelJson["category"].toString();

//   int n_outs = modelJson["dynamic_outputs"].toInt();
//   if(n_outs > 0)
//   {
//     children_list_.resize(n_outs);
//   }

  Init();
  setNodeName(node_.name);
}