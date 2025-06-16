#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" 
#include "std_msgs/msg/int8.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <queue>
#include <array>


#include "../include/wiseglove18/Wiseglove.h"


class GloveDataPub : public rclcpp::Node
{
    
public:
    GloveDataPub() : Node("glove_data_pub")
    {
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("angle_data", 10);
        scaled_data_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("scaled_data", 10);
        dataOrg_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("dataOrg_data", 10);
        quat_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quat_data", 10);
        quatOrg_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quatOrg_data", 10);
        
        glove_calib_subscriber_ = this->create_subscription<std_msgs::msg::Int8>("glove_calib_cmd",
            10, std::bind(&GloveDataPub::glove_calib_callback, this, std::placeholders::_1)
        );

        glove_imu_calib_subscriber_ = this->create_subscription<std_msgs::msg::Int8>("glove_imu_calib_cmd",
            10, 
            [this](const std_msgs::msg::Int8::SharedPtr msg)
            {
                if (msg->data == 1)
                {
                    m_glove->ResetQuat();
                }
            }
        );

        timer_10hz = this->create_wall_timer(
            std::chrono::milliseconds(100),
            // std::chrono::microseconds(12500),
            std::bind(&GloveDataPub::timer_callback_10, this));

        timer_50hz = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&GloveDataPub::timer_callback_50, this));
            
        keyboard_thread_ = std::thread(&GloveDataPub::keyboard_input_thread, this);
        char port[] = "/dev/ttyUSB0";
        if (wg_init(port))
        {
            RCLCPP_INFO(this->get_logger(), "wiseglove init success!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "wiseglove init failed!");
            rclcpp::shutdown();
        }
 
    }


    ~GloveDataPub()
    {

        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }


private:

    int wg_init(char* port)
    {

        m_glove = new Wiseglove();

        if (m_glove == NULL) return 0;

        bool isopen = m_glove->Open(port, 115200);
        if (!isopen)
        {
            printf("open port error\n");
            return 0;
        }

        bool sn_ret = m_glove->GetSn(sn);
        if (sn_ret)
        {
            RCLCPP_INFO(this->get_logger(),"sn:%s\n", sn);
        }
        bool model_ret = m_glove->GetModel(model);
        if (model_ret)
        {
            RCLCPP_INFO(this->get_logger(),"model:%s\n", model);
        }
        return 1;
    }

    unsigned int wg_getdata(unsigned short* data)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetData(data);
        return timestamp;
    }

    unsigned int wg_getscaleddata(unsigned short* sc_data)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetData(data_Org);
        if(timestamp>0)
        {
            if(calib_flag)
            {
                update_calibdata(max_data, min_data, data_Org);
            }
            for (int i = 0; i < 18; i++)
            {

                if (data_Org[i] > max_data[i])
                {
                    data_Org[i] = max_data[i];
                }
                if (data_Org[i] < min_data[i])
                {
                    data_Org[i] = min_data[i];
                }

                sc_data[i] = (data_Org[i] - min_data[i]) * 1000.0 / (max_data[i] - min_data[i]);
            }
        }
        return timestamp;
    }

    unsigned int wg_getangle(float* angle)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetAngle(angle);
        return timestamp;
    }

    unsigned int wg_getQuat(float* quat)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetQuat(quat);
        return timestamp;
    }

    unsigned int wg_getQuatOrg(float* quat)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetQuatOrg(quat);
        return timestamp;
    }
    
    void timer_callback_50()
    {
        std_msgs::msg::Float32MultiArray angle_msg;
        angle_msg.data.clear();  // 清空数据
        timestamp = wg_getangle(angle);
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                angle_msg.data.push_back(angle[i]);
            }
            angle_publisher_->publish(angle_msg);
            if (timestamp > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Angle Data:");
                for (int i = 0; i < 18; i++)
                {
                    std::cout << std::fixed << std::setprecision(1) <<angle[i]<<" ";
                }
                std::cout<<std::endl;
            }
        }

        std_msgs::msg::Int32MultiArray scaled_data_msg;
        scaled_data_msg.data.clear();  // 清空数据
        timestamp = wg_getscaleddata(scaled_data);
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                scaled_data_msg.data.push_back(scaled_data[i]);
            }
            scaled_data_publisher_->publish(scaled_data_msg);

            RCLCPP_INFO(this->get_logger(), "Scaled Data:");
            for (int i = 0; i < 18; i++)
            {
                std::cout<<scaled_data[i]<<" ";
            }
            std::cout<<std::endl;
        }

        std_msgs::msg::Int32MultiArray dataOrg_msg;
        dataOrg_msg.data.clear();  // 清空数据
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                dataOrg_msg.data.push_back(data_Org[i]);
            }
            dataOrg_publisher_->publish(dataOrg_msg);

            RCLCPP_INFO(this->get_logger(), "Data:");
            for (int i = 0; i < 18; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << data_Org[i] << " ";
            }
            std::cout<<std::endl;
        }


        if (show_calib_data)
        {
            RCLCPP_INFO(this->get_logger(), "--------------------------------");
            RCLCPP_INFO(this->get_logger(), "Max:");
            for (int i = 0; i < 18; i++)
            {
                std::cout<< max_data[i] << " ";
            }
            std::cout<<std::endl;
            RCLCPP_INFO(this->get_logger(), "Min:");
            for (int i = 0; i < 18; i++)
            {
                std::cout <<min_data[i] << " ";
            }
            std::cout<<std::endl;

            RCLCPP_INFO(this->get_logger(), "--------------------------------");
        }
    }

    void timer_callback_10()
    {
        std_msgs::msg::Float32MultiArray quat_msg;
        quat_msg.data.clear();  // 清空数据
        timestamp = wg_getQuat(quat);
        if (timestamp > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                quat_msg.data.push_back(quat[i]);
            }
            quat_publisher_->publish(quat_msg);

            RCLCPP_INFO(this->get_logger(), "Quat Data:");
            for (int i = 0; i < 16; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << quat[i] << " ";
            }
            std::cout<<std::endl;

        }
        std_msgs::msg::Float32MultiArray quatOrg_msg;
        quatOrg_msg.data.clear();  // 清空数据
        timestamp = wg_getQuatOrg(quatOrg);
        if (timestamp > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                quatOrg_msg.data.push_back(quatOrg[i]);
            }
            quatOrg_publisher_->publish(quatOrg_msg);

            RCLCPP_INFO(this->get_logger(), "QuatOrg Data:");
            for (int i = 0; i < 16; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << quatOrg[i] << " ";
            }
            std::cout<<std::endl;
        }
    }


    int kbhit(void)
    {
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    void glove_calib_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if(msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Glove Calib Command Received");
            for(int i = 0; i < 18; i++)
            {
                max_data[i] = 0;
                min_data[i] = USHRT_MAX;
            }
            calib_flag = true;
            show_calib_data = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Glove Calib Command Stop");
            calib_flag = false;
            show_calib_data = false;
        }
    }

    void keyboard_input_thread()
        {
            while (rclcpp::ok())  
            {
                if (kbhit())
                {
                    char ch = getchar();
                    switch (ch)
                    {
                        case 'q':
                            std::cout << "Exiting..." << std::endl;
                            rclcpp::shutdown();  
                            return;
                        case 'r':
                            // m_glove->ResetCalib();
                            for(int i = 0; i < 18; i++)
                            {
                                max_data[i] = 0;
                                min_data[i] = USHRT_MAX;
                            }
                            calib_flag = true;
                            show_calib_data = true;
                            break;
                        case 't':
                            RCLCPP_INFO(this->get_logger(), "Calib Complete");
                            calib_flag = false;
                            show_calib_data = false;
                            break;
                        case 'a':
                            m_glove->ResetQuat();
                            break;
                        default:
                            break;
                    }
                }

                usleep(10000);  // 10ms
            }
        }

    void update_calibdata(unsigned short* max_data, unsigned short* min_data,unsigned short* dataOrg,size_t size = 50)
    {
        if (max_vec.size() < size)
        {
            std::array<unsigned short, 18> max_data_;
            for (int i = 0; i < 18; i++)
            {
                max_data_[i] = dataOrg[i];
            }
            max_vec.push_back(max_data_);
        }
        if (min_vec.size() < size)
        {
            std::array<unsigned short, 18> min_data_;
            for (int i = 0; i < 18; i++)
            {
                if (data_Org[i] < 200.0){continue;}
                min_data_[i] = dataOrg[i];
            }
            min_vec.push_back(min_data_);
        }
        std::array<unsigned short, 18> max_data_avg;
        std::array<unsigned short, 18> min_data_avg;
        for(size_t i = 0; i < 18 ; i++)
        {
            int max_data_sum = 0;
            int min_data_sum = 0;
            for (size_t j = 0; j < max_vec.size(); j++)
            {
                max_data_sum += max_vec[j][i];
            }
            max_data_avg[i] = max_data_sum / max_vec.size();
            if (max_data_avg[i] > max_data[i])
            {
                max_data[i] = max_data_avg[i];
            }
            for(size_t j = 0; j < min_vec.size(); j++)
            {
                min_data_sum += min_vec[j][i];
            }
            min_data_avg[i] = min_data_sum / min_vec.size();
            if (min_data_avg[i] < min_data[i])
            {
                min_data[i] = min_data_avg[i];
            }
        }
        if(max_vec.size() >= size)
        {
            max_vec.erase(max_vec.begin());
        }
        if(min_vec.size() >= size)
        {
            min_vec.erase(min_vec.begin());
        }

        for(int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 18; j++)
            {
                std::cout <<std::setw(3)<< max_vec[i][j] << " ";
            }
            std::cout<<std::endl;
        }
        for(int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 18; j++)
            {
                std::cout <<std::setw(3)<< min_vec[i][j] << " ";
            }
            std::cout<<std::endl;
        }
    }


    Wiseglove* m_glove = NULL;
    unsigned short scaled_data[18] = {0};
    unsigned short data_Org[18] = {0};

    unsigned short max_data[18] = {0};
    unsigned short min_data[18] = {USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX};

    std::vector<std::array<unsigned short, 18>> max_vec;
    std::vector<std::array<unsigned short, 18>> min_vec;


    bool calib_flag = false;
    bool show_calib_data = false;

    float angle[18] = {0};
    float quat[16] = {0};
    float quatOrg[16] = {0};
    char model[20];
    char sn[20];
    unsigned int timestamp = 0;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr scaled_data_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr dataOrg_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quat_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quatOrg_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr glove_calib_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr glove_imu_calib_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_10hz;
    rclcpp::TimerBase::SharedPtr timer_50hz;
    std::thread keyboard_thread_; 

};





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GloveDataPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
