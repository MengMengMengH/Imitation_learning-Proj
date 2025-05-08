#include <thread>
#include <QApplication>
#include <memory>
#include "qt_mainwin_control/main_window.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qt_control");
    QApplication app(argc, argv);
    MainWindow main_window(node);
    main_window.show();

    int result = app.exec();
    
    rclcpp::shutdown();
   
    return result;
}