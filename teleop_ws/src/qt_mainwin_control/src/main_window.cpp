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
    glove_calib_state_ = !glove_calib_state_;
    if(glove_calib_state_)
    {
        ui->gloveCalib_button->setText("结束标定");
        ui->gloveCalib_button->setStyleSheet("background-color: red;color : white");
    }
    else
    {
        ui->gloveCalib_button->setText("手套标定");
        ui->gloveCalib_button->setStyleSheet("background-color: green; color : white");
    }
    publish_cmd(glove_calib_pub_cmd_,glove_calib_state_ ? 1 : 0);
}

void MainWindow::on_gloveIMUCalib_button_clicked()
{
    ui->gloveIMUCalib_button->setStyleSheet("background-color: green; color : white");
    publish_cmd(glove_imu_calib_pub_cmd_ , 1);
}

void MainWindow::on_inspireCalib_button_clicked()
{
    publish_cmd(inspire_calib_pub_cmd_,3);
    ui->inspireCalib_button->setText("力传感器标定中");
    ui->inspireCalib_button->setStyleSheet("background-color: gray; color: white");
    ui->inspireCalib_button->setEnabled(false);

    QTimer::singleShot(10000, this, [this]() 
    {
        ui->inspireCalib_button->setText("灵巧手传感器标定");
        ui->inspireCalib_button->setStyleSheet("background-color: green; color: white");
        ui->inspireCalib_button->setEnabled(true);
    });
}

void MainWindow::publish_cmd(rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_,const int8_t cmd)
{
    auto msg = std_msgs::msg::Int8();
    msg.data = cmd;
    pub_->publish(msg);
}