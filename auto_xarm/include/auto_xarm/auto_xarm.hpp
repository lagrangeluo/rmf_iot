#ifndef AUTO_XARM
#define AUTO_XARM

//std include
#include <string>
#include <mutex>
#include <memory>
#include <unistd.h> 
//ros2 include
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

//rmf msg include
#include "rmf_dispenser_msgs/msg/dispenser_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"

#include "rmf_ingestor_msgs/msg/ingestor_request.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"

//xarm service include
#include "xarm_api/xarm_msgs.h"
#include "xarm_api/xarm_ros_client.h"

namespace agx_xarm
{
    class agx_xarm_client : public xarm_api::XArmROSClient,std::enable_shared_from_this<agx_xarm_client>
    {
        public:
            //using 
            using Dispenser_Requst_msgs = rmf_dispenser_msgs::msg::DispenserRequest;
            using Dispenser_Result_msgs = rmf_dispenser_msgs::msg::DispenserResult;
            using Ingestor_Request_msgs = rmf_ingestor_msgs::msg::IngestorRequest;
            using Ingestor_Result_msgs = rmf_ingestor_msgs::msg::IngestorResult;

            //init agx_xarm_client
            void init_client(rclcpp::Node::SharedPtr& node);
            

        private:
        //------ros2 node ------//
            std::shared_ptr<rclcpp::Node> _node;

            rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_sub;

            rclcpp::Subscription<Dispenser_Requst_msgs>::SharedPtr dispenser_request_sub;
            rclcpp::Subscription<Ingestor_Request_msgs>::SharedPtr ingestor_request_sub;
            rclcpp::Publisher<Dispenser_Result_msgs>::SharedPtr dispenser_result_pub;
            rclcpp::Publisher<Ingestor_Result_msgs>::SharedPtr ingestor_result_pub;

            Dispenser_Requst_msgs current_dis_request;
            Ingestor_Request_msgs current_ing_request;


        //-------agx xarm msgs-------//

            enum agx_xarm_status_type{
                READY,
                DISPENSING,
                DISPENSED,
                GRIPPING,
                GRIPPED,
                INGESTING,
                INGESTED,
                OPENING,
                OPENED,
            }agx_xarm_status;

            enum agx_xarm_position_status_type{
                ARRIVED,
                ARRIVING
            }agx_xarm_position_status;

        //--------agx xarm methods-----------//
        void update_robot_state_callback(std::shared_ptr<xarm_msgs::msg::RobotMsg> msg);
       
        void dispenser_request_callback(std::shared_ptr<Dispenser_Requst_msgs> msg);

        void ingestor_requests_callback(std::shared_ptr<Ingestor_Request_msgs> msg);

        void update_robot_state(void);

        typedef struct TRAVEL_INFO_S
        {
            std::vector<_Float32> pose;
            _Float32 speed;
            _Float32 acc;
            _Float32 mvtime;
        }TRAVEL_INFO_S;

        //point to fetch objects
        struct
        {
            _Float32 x = 36.5169;
            _Float32 y = 131.985;
            _Float32 z = 257.264;
            _Float32 x_r = 3.04817;
            _Float32 y_r = -0.00899858;
            _Float32 z_r = 1.92357;
        }xarm_target_pose;

        //point to hold on
        struct
        {
            _Float32 x = 218.115;
            _Float32 y = -6.8665;
            _Float32 z = 331.157;
            _Float32 x_r = 2.9618;
            _Float32 y_r = -0.155483;
            _Float32 z_r = 0.373316;
        }xarm_init_pose;

        struct
        {
            _Float32 x = 18.0673;
            _Float32 y = 152.474;
            _Float32 z = 434.077;
            _Float32 x_r = 2.9152;
            _Float32 y_r = -0.0127551;
            _Float32 z_r = 1.50147;
        }xarm_hold_pose;


        //point to put the object
        struct
        {
            _Float32 x = -153.233;
            _Float32 y = 130.263;
            _Float32 z = 257.859;
            _Float32 x_r = 2.96533;
            _Float32 y_r = -0.146458;
            _Float32 z_r = 2.22903;
        }xarm_put_pose;

        TRAVEL_INFO_S travel_info_last, travel_info_current,dispenser_target, ingestor_target, init_target, hold_target;
        
        //-----------------xarm_gripper service client

        //how to use? like this:
        //auto result_future = open_lite6_gripper_client->async_send_request(open_lite6_gripper_req);

        rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr open_lite6_gripper_client;
        std::shared_ptr<xarm_msgs::srv::Call::Request> open_lite6_gripper_req;

        rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr close_lite6_gripper_client;
        std::shared_ptr<xarm_msgs::srv::Call::Request> close_lite6_gripper_req;

        rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr stop_lite6_gripper_client;
        std::shared_ptr<xarm_msgs::srv::Call::Request> stop_lite6_gripper_req;

        rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr vacuum_control_client;
        std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> vacuum_control_request;

        bool use_vacuum = true;
        //-----------------------------------

    };//class agx_xarm_client
}//namespace agx_xarm
#endif