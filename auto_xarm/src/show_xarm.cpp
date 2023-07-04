#include "auto_xarm/show_xarm.hpp"


using namespace agx_xarm;


  void agx_xarm_client::init_client(rclcpp::Node::SharedPtr& node)
  {
    //init node ,publishers and subscribers
    _node = node;


      // about xarm robot states
       robot_state_sub = _node->create_subscription
      <xarm_msgs::msg::RobotMsg>(
        "/ufactory/robot_states",
        10,
        std::bind(&agx_xarm_client::update_robot_state_callback, this, std::placeholders::_1)
      );

      // init the dispense and the ingest position
      show_target.pose.push_back(xarm_init_pose.x);
      show_target.pose.push_back(xarm_init_pose.y);
      show_target.pose.push_back(xarm_init_pose.z);
      show_target.pose.push_back(xarm_init_pose.x_r);
      show_target.pose.push_back(xarm_init_pose.y_r);
      show_target.pose.push_back(xarm_init_pose.z_r);
      //init_target.speed = 30;
      //init_target.acc = 500;
      //init_target.mvtime = 10;

      hold_target.pose.push_back(xarm_hold_pose.x);
      hold_target.pose.push_back(xarm_hold_pose.y);
      hold_target.pose.push_back(xarm_hold_pose.z);
      hold_target.pose.push_back(xarm_hold_pose.x_r);
      hold_target.pose.push_back(xarm_hold_pose.y_r);
      hold_target.pose.push_back(xarm_hold_pose.z_r);

      //load the fetch point
      dispenser_target.pose.push_back(xarm_target_pose.x);
      dispenser_target.pose.push_back(xarm_target_pose.y);
      dispenser_target.pose.push_back(xarm_target_pose.z);
      dispenser_target.pose.push_back(xarm_target_pose.x_r);
      dispenser_target.pose.push_back(xarm_target_pose.y_r);
      dispenser_target.pose.push_back(xarm_target_pose.z_r);
      //dispenser_target.speed = 30;
      //dispenser_target.acc = 100;

      ingestor_target.pose.push_back(xarm_put_pose.x);
      ingestor_target.pose.push_back(xarm_put_pose.y);
      ingestor_target.pose.push_back(xarm_put_pose.z);
      ingestor_target.pose.push_back(xarm_put_pose.x_r);
      ingestor_target.pose.push_back(xarm_put_pose.y_r);
      ingestor_target.pose.push_back(xarm_put_pose.z_r);
      //ingestor_target.speed = 20;
      //ingestor_target.acc = 100;
      //init_target.mvtime = 10;


      //auto initial = set_position(show_target.pose,show_target.speed,show_target.acc,show_target.mvtime);

      //init gripper service 
      stop_lite6_gripper_client = _node->create_client<xarm_msgs::srv::Call>("ufactory/stop_lite6_gripper");
      close_lite6_gripper_client = _node->create_client<xarm_msgs::srv::Call>("ufactory/close_lite6_gripper");
      open_lite6_gripper_client = _node->create_client<xarm_msgs::srv::Call>("ufactory/open_lite6_gripper");
      vacuum_control_client = _node->create_client<xarm_msgs::srv::VacuumGripperCtrl>("ufactory/set_vacuum_gripper");


      open_lite6_gripper_req = std::make_shared<xarm_msgs::srv::Call::Request>();
      close_lite6_gripper_req = std::make_shared<xarm_msgs::srv::Call::Request>();
      stop_lite6_gripper_req = std::make_shared<xarm_msgs::srv::Call::Request>();
      vacuum_control_request = std::make_shared<xarm_msgs::srv::VacuumGripperCtrl::Request>();
      //init state machine
      agx_xarm_position_status = agx_xarm_position_status_type::ARRIVED;
      agx_xarm_status = agx_xarm_status_type::READY;
      agx_xarm_show_status = agx_xarm_show_status_type::OK;
      xarm_set_instruction = xarm_set_instruction_type::ENABLE;
      xarm_hold_mode = xarm_hold_type::HOLD;
      //sleep for init
      //sleep(8);
      std::cout<<" init completed! "<< agx_xarm_position_status;

  }

  void agx_xarm_client::update_robot_state_callback(std::shared_ptr<xarm_msgs::msg::RobotMsg> msg)
  {
    //反向迭代器
    auto first = msg->pose.begin();
    auto end = msg->pose.end();

    while(first != end)
    {
      travel_info_current.pose.push_back(*first);
      first++;
    }
    
    travel_info_last.pose.reserve(6); //allocate memory
    //compare current msg to the last one to decide should we cout this message
    if(travel_info_current.pose[0] == travel_info_last.pose[0]
          && travel_info_current.pose[1] == travel_info_last.pose[1]
          && travel_info_current.pose[2] == travel_info_last.pose[2]
          && travel_info_current.pose[3] == travel_info_last.pose[3]
          && travel_info_current.pose[4] == travel_info_last.pose[4]
          && travel_info_current.pose[5] == travel_info_last.pose[5])
    {
      // do nothing
    }
    else
    {
        //------------cout the messages----------------
        std::cout<<"we update robot states:";

        auto cout_first = travel_info_current.pose.begin();
        auto cout_end = travel_info_current.pose.end();
        
        while(cout_first != cout_end)
        {
          std::cout << " " << *cout_first;
          cout_first++;
        }
        std::cout<<" pos_state: "<< agx_xarm_position_status;
        std::cout<<" xarm_status: "<<agx_xarm_status<< std::endl;
        //---------------------------------------------
    }
    robot_show_time();

    travel_info_last=travel_info_current; //record the last messages
    travel_info_current.pose.clear();
   

  }

  void agx_xarm_client::robot_show_time(void)
  {
    if(travel_info_current.pose[0] == show_target.pose[0]
          && travel_info_current.pose[1] == show_target.pose[1]
          && travel_info_current.pose[2] == show_target.pose[2]
          && travel_info_current.pose[3] == show_target.pose[3]
          && travel_info_current.pose[4] == show_target.pose[4]
          && travel_info_current.pose[5] == show_target.pose[5])
    {
        agx_xarm_position_status = ARRIVED;
        xarm_set_instruction = xarm_set_instruction_type::ENABLE;
    }
    else
    {
        agx_xarm_position_status = ARRIVING;
        if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
        {
            std::cout<<"send a target!"<<std::endl;           
            auto initial = set_position(show_target.pose,show_target.speed,show_target.acc,show_target.mvtime);
            
            xarm_set_instruction = xarm_set_instruction_type::DISABLE;
        }
    }

    //sleep for next loop
    if(agx_xarm_show_status == agx_xarm_show_status_type::OK)
    {
        set_target_pose(show_target,xarm_init_pose);
        agx_xarm_show_status = agx_xarm_show_status_type::GOING;
        return;
    }
    //begin loop
    if(agx_xarm_show_status == agx_xarm_show_status_type::GOING)
    {
        robot_excute_once(xarm_fetch_pose);
    }
  }

    void agx_xarm_client::robot_excute_once(XARM_TARGET_POSE_T poes)
    {
        if(agx_xarm_status == agx_xarm_status_type::READY)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                if(xarm_hold_mode == xarm_hold_type::HOLD)
                {
                  poes.z += z_hold_low;
                  poes.x += (x_step*loop_count);
                  if(reverse == true)
                    poes.y -= step_forward;
                  set_target_pose(show_target,poes);
                  xarm_hold_mode = xarm_hold_type::DISHOLD;

                  return;
                }
                if(xarm_hold_mode == xarm_hold_type::DISHOLD)
                {
                  poes.x += (x_step*loop_count);
                  if(reverse == true)
                    poes.y -= step_forward;
                  set_target_pose(show_target,poes);
                  xarm_hold_mode = xarm_hold_type::HOLD;
                }
                agx_xarm_status = agx_xarm_status_type::DISPENSING;

            }
            return;
        }

        if(agx_xarm_status == agx_xarm_status_type::DISPENSING)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                if(use_vacuum == true)
                    {
                    vacuum_control_request->on = true;
                    vacuum_control_request->wait = false;
                    vacuum_control_request->timeout = 0.0;
                    vacuum_control_request->delay_sec = 0.0;
                    
                    auto result_future = vacuum_control_client->async_send_request(vacuum_control_request);
                    }
                else
                    auto result_future = close_lite6_gripper_client->async_send_request(close_lite6_gripper_req);
                
                sleep(1);
                agx_xarm_status = agx_xarm_status_type::DISPENSED;
            }
            return;
        }
        if(agx_xarm_status == agx_xarm_status_type::DISPENSED)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                poes.x += (x_step*loop_count);
                poes.z += z_hold_high;
                  if(reverse == true)
                    poes.y -= step_forward;
                set_target_pose(show_target,poes);
                agx_xarm_status = agx_xarm_status_type::GRIPPING;
            }
            return;
        }
        if(agx_xarm_status == agx_xarm_status_type::GRIPPING)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                poes.x += (x_step*loop_count - 3.5);
                poes.z += z_hold_high;
                if(reverse == false)
                  poes.y -= step_forward;
                set_target_pose(show_target,poes);
                agx_xarm_status = agx_xarm_status_type::GRIPPED;
            }
            return;
        }
        if(agx_xarm_status == agx_xarm_status_type::GRIPPED)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                poes.x += (x_step*loop_count - 3.5);
                if(reverse == false)
                  poes.y -= step_forward;
                set_target_pose(show_target,poes);
                agx_xarm_status = agx_xarm_status_type::INGESTING;
            }
            return;
        }
        if(agx_xarm_status == agx_xarm_status_type::INGESTING)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                if(use_vacuum == true)
                    {
                    vacuum_control_request->on = false;
                    vacuum_control_request->wait = false;
                    vacuum_control_request->timeout = 0.0;
                    vacuum_control_request->delay_sec = 0.0;
                    
                    auto result_future = vacuum_control_client->async_send_request(vacuum_control_request);
                    }
                else
                    auto result_future = open_lite6_gripper_client->async_send_request(open_lite6_gripper_req);
                sleep(1);

                poes.x += (x_step*loop_count);
                poes.z += z_hold_high;
                if(reverse == false)
                  poes.y -= step_forward;
                set_target_pose(show_target,poes);
                agx_xarm_status = agx_xarm_status_type::INGESTED;
            }
            return;
        }
        if(agx_xarm_status == agx_xarm_status_type::INGESTED)
        {
            if(xarm_set_instruction == xarm_set_instruction_type::ENABLE)
            {
                set_target_pose(show_target,xarm_init_pose);
                agx_xarm_status = agx_xarm_status_type::OPENING;
                
                //循环次数+1
                loop_count++;

                //循环结束，初始化标识符
                if(loop_count == loop_time)
                {
                  loop_count = 0;
                  reverse = !reverse;
                }
                agx_xarm_status = agx_xarm_status_type::READY;
                agx_xarm_show_status = agx_xarm_show_status_type::OK;
            }
            return;
        }
    }

    void agx_xarm_client::set_target_pose(TRAVEL_INFO_S &t,XARM_TARGET_POSE_T &pose)
    {
        t.pose.clear();
        t.pose.push_back(pose.x);
        t.pose.push_back(pose.y);
        t.pose.push_back(pose.z);
        t.pose.push_back(pose.x_r);
        t.pose.push_back(pose.y_r);
        t.pose.push_back(pose.z_r);
    }

/*
  void agx_xarm_client::update_robot_state(void)
  {
    //if(!travel_info_current.pose.empty() && !dispenser_target.pose.empty())
    //{
      rclcpp::WallRate loop_rate(0.5);
      rclcpp::WallRate loop_rate_fast(1);

      
      if(agx_xarm_status == DISPENSING)
      {
        if(travel_info_current.pose[0] == dispenser_target.pose[0]
          && travel_info_current.pose[1] == dispenser_target.pose[1]
          && travel_info_current.pose[2] == dispenser_target.pose[2]
          && travel_info_current.pose[3] == dispenser_target.pose[3]
          && travel_info_current.pose[4] == dispenser_target.pose[4]
          && travel_info_current.pose[5] == dispenser_target.pose[5])
          {
            agx_xarm_position_status = ARRIVED;
            std::cout<<"dispensing target arrived"<<std::endl;
          }
        else
          {
            agx_xarm_position_status = ARRIVING;
          }
      }

      if(agx_xarm_status == INGESTING)
      {
        if(travel_info_current.pose[0] == ingestor_target.pose[0]
          && travel_info_current.pose[1] == ingestor_target.pose[1]
          && travel_info_current.pose[2] == ingestor_target.pose[2]
          && travel_info_current.pose[3] == ingestor_target.pose[3]
          && travel_info_current.pose[4] == ingestor_target.pose[4]
          && travel_info_current.pose[5] == ingestor_target.pose[5])
          {
            agx_xarm_position_status = ARRIVED;
            std::cout<<"ingesting target arrived"<<std::endl;
          }
          else
          {
            agx_xarm_position_status = ARRIVING;
          }
      }
   // }

    if(agx_xarm_status == agx_xarm_status_type::DISPENSING)
    {
      if(agx_xarm_position_status == ARRIVED)
      {
        agx_xarm_status = agx_xarm_status_type::DISPENSED;
        std::cout<<"current state: DISPENSED"<<std::endl;
      }
    }
    if(agx_xarm_status == agx_xarm_status_type::DISPENSED)
    {
      if(use_vacuum == true)
        {
          vacuum_control_request->on = true;
          vacuum_control_request->wait = false;
          vacuum_control_request->timeout = 0.0;
          vacuum_control_request->delay_sec = 0.0;
          
          auto result_future = vacuum_control_client->async_send_request(vacuum_control_request);
        }
      else
         auto result_future = close_lite6_gripper_client->async_send_request(close_lite6_gripper_req);

      std::cout<<"opening the gripper"<<std::endl;
      agx_xarm_status = agx_xarm_status_type::GRIPPING;
      std::cout<<"current state: GRIPPING"<<std::endl;
      
      sleep(1);
      agx_xarm_status = agx_xarm_status_type::GRIPPED;
      std::cout<<"current state: GRIPPED"<<std::endl;


    }

    if(agx_xarm_status == agx_xarm_status_type::GRIPPED)
    {

      auto initial = set_position(hold_target.pose,hold_target.speed,hold_target.acc,hold_target.mvtime);

      sleep(0.5);
      Dispenser_Result_msgs msg;
      msg.time.sec = 1;
      msg.time.nanosec = 0;
      msg.request_guid = current_dis_request.request_guid;
      msg.source_guid = current_dis_request.target_guid;
      msg.status = 1;

      dispenser_result_pub->publish(msg);

      std::cout<<"DISPENSING task is done!"<<std::endl;
    }

    if(agx_xarm_status == agx_xarm_status_type::INGESTING)
    {
      if(agx_xarm_position_status == ARRIVED)
      {
        agx_xarm_status = agx_xarm_status_type::INGESTED; 
        std::cout<<"current state: INGESTED"<<std::endl;
      }
    }

    if(agx_xarm_status == agx_xarm_status_type::INGESTED)
    {

      if(use_vacuum == true)
        {
          vacuum_control_request->on = false;
          vacuum_control_request->wait = false;
          vacuum_control_request->wait = 0.0;
          vacuum_control_request->delay_sec = 0.0;
          auto result_future = vacuum_control_client->async_send_request(vacuum_control_request);
        }
      else
        auto result_future = open_lite6_gripper_client->async_send_request(open_lite6_gripper_req);

      std::cout<<"opening the gripper"<<std::endl;
      agx_xarm_status = agx_xarm_status_type::OPENING;
      std::cout<<"current state: OPENING"<<std::endl;

      sleep(0.8);

      auto initial = set_position(hold_target.pose,hold_target.speed,hold_target.acc,hold_target.mvtime);

      sleep(2.5);

      Ingestor_Result_msgs msg_i;
      msg_i.time.sec = 1;
      msg_i.time.nanosec = 0;
      msg_i.request_guid = current_ing_request.request_guid;
      msg_i.source_guid = current_ing_request.target_guid;
      msg_i.status = 1;

      ingestor_result_pub->publish(msg_i);

      agx_xarm_status = agx_xarm_status_type::OPENED;
      std::cout<<"current state: OPENED"<<std::endl;
    }

    if(agx_xarm_status == agx_xarm_status_type::OPENED)
    {
      if(use_vacuum == true)
          auto result_future = stop_lite6_gripper_client->async_send_request(stop_lite6_gripper_req);

      auto initial = set_position(init_target.pose,init_target.speed,init_target.acc,init_target.mvtime);

      agx_xarm_status = READY;
      std::cout<<"current state: READY"<<std::endl;
    }
  }
*/

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<agx_xarm_client> xarm_client_node = std::make_shared<agx_xarm_client>();
    auto node_agx = rclcpp::Node::make_shared("agx_xarm_node");
    auto node_xarm = rclcpp::Node::make_shared("xarm_api_client");

    xarm_client_node->init(node_xarm,"ufactory"); //init xarm_api_client node
    xarm_client_node->init_client(node_agx); // init agx_xarm node
    std::cout<<"init complete!"<<std::endl;

    rclcpp::spin(node_agx); //a thread to spin agx_xarm node

    
    return 0;
  }