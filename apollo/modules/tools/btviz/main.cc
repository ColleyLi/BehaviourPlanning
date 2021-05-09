#include "mainwindow.h"
#include "cyber/cyber.h"

#include <QtWidgets/QApplication>

int main(int argc, char* argv[])
{
    QApplication qt_app(argc, argv);
    qt_app.setApplicationName("BTViz");
    qt_app.setWindowIcon(QPixmap(":images/btviz_logo.png"));
    
    apollo::cyber::Init(argv[0]);

    MainWindow main_window;
    main_window.show();
    
    return qt_app.exec();
}