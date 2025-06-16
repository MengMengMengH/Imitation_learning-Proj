#ifndef QTMAINWINCONTROL__MAIN_WINDOW_HPP_
#define QTMAINWINCONTROL__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr node,QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_gloveCalib_button_clicked();
    void on_gloveIMUCalib_button_clicked();
    void on_inspireCalib_button_clicked();

private:
    void publish_cmd(rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_,const int8_t cmd);
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr glove_calib_pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr glove_imu_calib_pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr inspire_calib_pub_cmd_;
    std::thread ros_spin_thread_;

    bool glove_calib_state_ = false;
    bool inspire_calib_state_ = false;

};




#endif  // QTMAINWINCONTROL__MAIN_WINDOW_HPP_