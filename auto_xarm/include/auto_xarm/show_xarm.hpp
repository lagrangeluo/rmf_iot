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

//xarm service include
#include "xarm_api/xarm_msgs.h"
#include "xarm_api/xarm_ros_client.h"

namespace agx_xarm
{
    class agx_xarm_client : public xarm_api::XArmROSClient,std::enable_shared_from_this<agx_xarm_client>
    {
        public:
            //init agx_xarm_client
            void init_client(rclcpp::Node::SharedPtr& node);
            

        private:
        //------ros2 node ------//
            std::shared_ptr<rclcpp::Node> _node;

            rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_sub;

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

            enum agx_xarm_show_status_type{
                OK,
                GOING,
                END
            }agx_xarm_show_status;

            enum xarm_set_instruction_type{
                ENABLE,
                DISABLE,
            }xarm_set_instruction;

            enum xarm_hold_type{
                HOLD,
                DISHOLD,
            }xarm_hold_mode;
        //--------agx xarm methods-----------//
        void update_robot_state_callback(std::shared_ptr<xarm_msgs::msg::RobotMsg> msg);

        void update_robot_state(void);

        void robot_show_time(void);

        struct TRAVEL_INFO_S
        {
            std::vector<_Float32> pose;
            _Float32 speed;
            _Float32 acc;
            _Float32 mvtime;
        };

        struct XARM_TARGET_POSE_T
        {
            _Float32 x = 0.0;
            _Float32 y = 0.0;
            _Float32 z = 0.0;
            _Float32 x_r = 0.0;
            _Float32 y_r = 0.0;
            _Float32 z_r = 0.0;
        };

        void robot_excute_once(XARM_TARGET_POSE_T poes);
        void set_target_pose(TRAVEL_INFO_S &t,XARM_TARGET_POSE_T &pose);

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

        // XARM_TARGET_POSE_T xarm_init_pose = {-161.1,16.9028,321.892,3.01423,0.00913622,-2.85013};
        // XARM_TARGET_POSE_T xarm_fetch_pose = {-212.369,143.135,258.2,3.04526,0.0804873,-2.91201};
        // XARM_TARGET_POSE_T xarm_init_pose = {-161.1,16.9028,321.892,3.14,0.001,-3.14};
        // XARM_TARGET_POSE_T xarm_fetch_pose = {-212.369,143.135,256.2,3.14,0.001,-3.14};

        XARM_TARGET_POSE_T xarm_init_pose = {-161.1,16.9028,321.892,3.014,0.08,-3.14};
        XARM_TARGET_POSE_T xarm_fetch_pose = {-212.369,143.135,256.2,3.014,0.08,-3.14};

        //point to hold on
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

        struct
        {
            _Float32 x = 0.0;
            _Float32 y = 0.0;
            _Float32 z = 0.0;
            _Float32 x_r = 0.0;
            _Float32 y_r = 0.0;
            _Float32 z_r = 0.0;
        }xarm_goal_pose;
        

        TRAVEL_INFO_S travel_info_last, travel_info_current,dispenser_target, ingestor_target, init_target, hold_target;
        TRAVEL_INFO_S show_target;
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
        //-----------------------------------
        
        //是否使用气动吸盘
        bool use_vacuum = true;

        //循环次数，即有多少个方块需要夹取，count记录当前夹取的个数
        uint16_t loop_time = 3;
        uint16_t loop_count = 0;

        //x，y，z方向上的间隔
        uint16_t x_step = 65;
        uint16_t step_forward = 100;
        uint16_t z_hold_low = 50;
        uint16_t z_hold_high = 100;

        bool reverse = false;
    };//class agx_xarm_client

}//namespace agx_xarm
#endif