#include <thread>
#include "qt_mainwin_control/main_window.hpp"
#include "ui_main_window.h"
#include <QMessageBox>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    ros_node_(node)
{
    ui->setupUi(this);
    glove_calib_pub_cmd_ = ros_node_->create_publisher<std_msgs::msg::Int8>("glove_calib_cmd", 10);
    glove_imu_calib_pub_cmd_ = ros_node_->create_publisher<std_msgs::msg::Int8>("glove_imu_calib_cmd", 10);
    inspire_calib_pub_cmd_ = ros_node_->create_publisher<std_msgs::msg::Int8>("inspire_calib_cmd", 10);

    ros_spin_thread_ = std::thread([this]()
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "ROS thread started");
        rclcpp::spin(this->ros_node_);
        RCLCPP_INFO(this->ros_node_->get_logger(), "ROS thread finished");
    });

    this->setWindowTitle("Imitation Learning GUI");
}

MainWindow::~MainWindow()
{
    if(ros_spin_thread_.joinable())
    {
        ros_spin_thread_.join();
    }
    delete ui;
}

void MainWindow::on_gloveCalib_button_clicked()
{
    publish_cmd(glove_calib_pub_cmd_,1);
}

void MainWindow::on_gloveIMUCalib_button_clicked()
{
    publish_cmd(glove_imu_calib_pub_cmd_,2);
}

void MainWindow::on_inspireCalib_button_clicked()
{
    publish_cmd(inspire_calib_pub_cmd_,3);
}

void MainWindow::publish_cmd(rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_,const int8_t& cmd)
{
    auto msg = std_msgs::msg::Int8();
    msg.data = cmd;
    pub_->publish(msg);
}