#include "btree_data_model.h"

#define MARGIN 10

BTreeDataModel::BTreeDataModel(const BTNode & node):
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
  main_layout_->setMargin(20);
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

BTreeDataModel::~BTreeDataModel()
{

}

void BTreeDataModel::Init()
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

  if (node_.category == "Tasks")
  {
    setNodeStyle(NodeStyle(
    R"(
    {
      "NodeStyle": 
      {
        "NormalBoundaryColor": [218, 232, 252],
        "SelectedBoundaryColor": [255, 165, 0],
        "GradientColor0": [218, 232, 252],
        "GradientColor1": [218, 232, 252],
        "GradientColor2": [218, 232, 252],
        "GradientColor3": [218, 232, 252],
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
  
  if (node_.category == "Selectors")
  {
    setNodeStyle(NodeStyle(
    R"(
    {
      "NodeStyle": 
      {
        "NormalBoundaryColor": [225, 213, 231],
        "SelectedBoundaryColor": [255, 165, 0],
        "GradientColor0": [225, 213, 231],
        "GradientColor1": [225, 213, 231],
        "GradientColor2": [225, 213, 231],
        "GradientColor3": [225, 213, 231],
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
  
  if (node_.category == "Checks")
  {
    setNodeStyle(NodeStyle(
    R"(
    {
      "NodeStyle": 
      {
        "NormalBoundaryColor": [213, 232, 212],
        "SelectedBoundaryColor": [255, 165, 0],
        "GradientColor0": [213, 232, 212],
        "GradientColor1": [213, 232, 212],
        "GradientColor2": [213, 232, 212],
        "GradientColor3": [213, 232, 212],
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

unsigned int BTreeDataModel::nPorts(PortType portType) const
{
  int result = 0;
  if( portType == QtNodes::PortType::Out)
  {
    if (node_.category == "Tasks" || node_.category == "Checks")
    {
      result = 0;
    }
    else
    {
      result = 1;
    }
  }
  else if( portType == QtNodes::PortType::In)
  {
    result = 1;
  }

  return result;
}

NodeDataModel::ConnectionPolicy BTreeDataModel::portOutConnectionPolicy(PortIndex) const
{
  return ConnectionPolicy::Many;
}

NodeDataType BTreeDataModel:: dataType(PortType, PortIndex) const
{
  return NodeDataType();
}

std::shared_ptr<NodeData> BTreeDataModel::outData(PortIndex)
{
  return nullptr;
}

QString BTreeDataModel::name() const
{
  return node_.name;
}

QWidget* BTreeDataModel::embeddedWidget()
{
  return main_widget_;
}

void BTreeDataModel::setNodeName(const QString& name)
{
    node_.name = name;
    node_name_line_edit_->setText(name);

    updateNodeSize();
    emit nodeNameChanged();
}

void BTreeDataModel::setNodeId(const QString& id)
{
  node_.id = id;
}

void BTreeDataModel::updateNodeSize()
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

QJsonObject BTreeDataModel::save() const
{
  QJsonObject node_json;

  node_json["name"] = node_.name;
  node_json["type"] = node_.type;
  node_json["id"] = node_.id;
  node_json["category"] = node_.category;

  return node_json;
}


void BTreeDataModel::restore(QJsonObject const &modelJson)
{
  node_.name = modelJson["name"].toString();
  node_.type = modelJson["type"].toString();
  node_.id = modelJson["id"].toString();
  node_.category = modelJson["category"].toString();

  Init();
  setNodeName(node_.name);
}