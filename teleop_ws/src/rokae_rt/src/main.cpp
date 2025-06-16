#include <iostream>
#include "rokae_rt/robot.h"

using namespace std;
using namespace rokae;

int main(int argc, char **argv)
{
    try
    {
        using namespace rokae;

        std::string ip = "192.168.0.160";
        std::error_code ec;
        rokae::xMateErProRobot robot(ip);
        std::cout<< "connection successful" << std::endl;
        robot.setOperateMode(OperateMode::automatic,ec);
        
    
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    // std::cout << "This is a placeholder for the fst_move example main function." << std::endl;
    // std::cout << "Please refer to the specific examples in the fst_move/src/example directory." << std::endl;
    return 0;
}