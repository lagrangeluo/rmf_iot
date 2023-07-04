#ifndef AGX_LIFT
#define AGX_LIFT

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "rmf_lift_msgs/msg/lift_request.hpp"
#include "rmf_lift_msgs/msg/lift_state.hpp"


class agx_lift : public rclcpp::Node
{
    public :

    using LIFT_STATE = rmf_lift_msgs::msg::LiftState;
    using LIFT_REQUEST = rmf_lift_msgs::msg::LiftRequest;
    using LIFT_STATE_Share = std::shared_ptr<LIFT_STATE>;
    using LIFT_REQUEST_Share = std::shared_ptr<LIFT_REQUEST>;

    agx_lift();

    private:

    rclcpp::Publisher<LIFT_STATE>::SharedPtr _state_pub;
    rclcpp::Subscription<LIFT_REQUEST>::SharedPtr _request_sub;

    LIFT_REQUEST current_request;
    LIFT_STATE curren_state;

    void lift_state_callback(LIFT_REQUEST_Share msg);

};

#endif //define AUTP_LIFT