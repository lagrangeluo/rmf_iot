#ifndef RMF_BUTTON
#define RMF_BUTTON

//std include
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

//lib include
#include <nlohmann/json.hpp>

//ros2 include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

//msg include
#include "rmf_iot_msg/msg/io_status.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_request.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"

//headfile include
#include "rmf_iot/serial_port.h"

using namespace std;
using namespace nlohmann;

namespace AgileX {

class rmf_button : public rclcpp::Node
{
public:
    //using 
    using Dispenser_Requst_msgs = rmf_dispenser_msgs::msg::DispenserRequest;
    using Dispenser_Result_msgs = rmf_dispenser_msgs::msg::DispenserResult;
    using Ingestor_Request_msgs = rmf_ingestor_msgs::msg::IngestorRequest;
    using Ingestor_Result_msgs = rmf_ingestor_msgs::msg::IngestorResult;
    using ApiTaskRequest_msgs = rmf_task_msgs::msg::ApiRequest;

    rmf_button(string node_name);
    void submit_task_request(void);
private:
    //uart
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;
    std::string port_name_;
    std::vector<uint8_t> uart_buffer;

    uint8_t receive_flag;
    // IO struct
    typedef struct
    {
        bool pin_1;
        bool pin_2;
        bool pin_3;
        bool pin_4;
    }IO_STATUS;
    IO_STATUS DI_Status,DO_Status;

    //button press flag
    bool button_press_flag = false;

    //uart api
    void connect(std::string dev_name, uint32_t bouadrate);
    void readData();
    void processRxData(uint8_t data);

    void GetAllDOStatus(void);
    void GetAllDIStatus(void);
    void pub_io_status(void);

    //ros
    rclcpp::Publisher<rmf_iot_msg::msg::IoStatus>::SharedPtr iostatus_pub;
    rmf_iot_msg::msg::IoStatus io_status;
    rclcpp::TimerBase::SharedPtr timer_;
    
    //rmf
    // rclcpp::Subscription<Dispenser_Requst_msgs>::SharedPtr dispenser_request_sub;
    // rclcpp::Subscription<Ingestor_Request_msgs>::SharedPtr ingestor_request_sub;
    rclcpp::Publisher<Dispenser_Requst_msgs>::SharedPtr dispenser_request_pub;
    rclcpp::Publisher<Ingestor_Request_msgs>::SharedPtr ingestor_request_pub;
    rclcpp::Publisher<ApiTaskRequest_msgs>::SharedPtr tast_request_pub;

    std::string json_msg_const;
    std::string request_id_const;
    int request_id;
    Dispenser_Requst_msgs current_dis_request;
    Ingestor_Request_msgs current_ing_request;
    ApiTaskRequest_msgs current_task_request;

    //json msgs
    json json_data;
    void convert_task_to_request_msg(const json& json_in,json& json_out);
};
}//namespace AgileX
#endif
