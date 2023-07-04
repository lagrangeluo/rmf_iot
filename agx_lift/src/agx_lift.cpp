#include "agx_lift/agx_lift.hpp"


 agx_lift::agx_lift()
 : Node("agx_lift_node")
{
  _state_pub =  this->create_publisher<LIFT_STATE>("/lift_states", 5);

  _request_sub = this->create_subscription
  <LIFT_REQUEST>(
    "/adapter_lift_requests",
    1,
    std::bind(&agx_lift::lift_state_callback, this, std::placeholders::_1)
  );
}

  void agx_lift::lift_state_callback(LIFT_REQUEST_Share msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("lift_state_callback"),
    "we receive a request !");

    current_request.session_id = msg->session_id;
    current_request.request_type = msg->request_type;
    current_request.destination_floor = msg->destination_floor;
    current_request.door_state = msg->door_state;
    current_request.lift_name = msg->lift_name;

    curren_state.current_floor = current_request.destination_floor;
    curren_state.door_state = current_request.door_state;
    curren_state.current_mode = current_request.request_type;
    curren_state.lift_name = current_request.lift_name;

    if(msg->request_type == 0)
    {
      curren_state.session_id = "\0";
    }
    else
    {
      curren_state.session_id = current_request.session_id;
    }

    _state_pub->publish(curren_state);

  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("agx_lift_node"),
      "starting init agx_lift_node!");
  std::shared_ptr<agx_lift> agx_lift_node = std::make_shared<agx_lift>();

  RCLCPP_INFO(rclcpp::get_logger("agx_lift_node"),
  "starting spin !");
  rclcpp::spin(agx_lift_node);

  return 0;
}
