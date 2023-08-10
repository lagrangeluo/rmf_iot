#include <cstdio>
#include <rmf_iot/rmf_button.hpp>

using namespace AgileX;
using namespace std;


  rmf_button::rmf_button(string node_name):Node(node_name)
{
  //init ros parameter

  blue_button_task = this->declare_parameter<std::string>("blue_button_task");
  yellow_button_task = this->declare_parameter<std::string>("yellow_button_task");
  green_button_task = this->declare_parameter<std::string>("green_button_task");
  red_button_task = this->declare_parameter<std::string>("red_button_task");
  
  //init uart buffer
  uart_buffer.reserve(8);

  //init uart publisher
  iostatus_pub = this->create_publisher<rmf_iot_msg::msg::IoStatus>("/io_status",10);
  timer_ = this->create_wall_timer(100ms, std::bind(&rmf_button::pub_io_status,this));

  //init rmf subscription
      //about rmf messages
      // dispenser_request_sub = _node->create_subscription
      // <Dispenser_Requst_msgs>(
      //   "/dispenser_requests",
      //   10,
      //   std::bind(&agx_xarm_client::dispenser_request_callback, this, std::placeholders::_1)
      // );
      
      // ingestor_request_sub = _node->create_subscription
      // <Ingestor_Request_msgs>(
      //   "/ingestor_requests",
      //   10,
      //   std::bind(&agx_xarm_client::ingestor_requests_callback, this, std::placeholders::_1)
      // );
  //init rmf publisher
    dispenser_request_pub = this->create_publisher
        <Dispenser_Requst_msgs>("/dispenser_requests", 5);

    ingestor_request_pub = this->create_publisher
        <Ingestor_Request_msgs>("/ingestor_requests", 5);
    tast_request_pub = this->create_publisher
        <ApiTaskRequest_msgs>("/task_api_requests", rclcpp::SystemDefaultsQoS().reliable().transient_local());

  //init rmf request
    current_dis_request.time.sec = 1;
    current_dis_request.time.nanosec = 1;
    current_dis_request.request_guid = "delivery.dispatch-0";
    current_dis_request.target_guid = "pickup";

    request_id_const = "agx_rmf_task-";
    request_id = 1;

  //init json msgs
  std::ifstream json_file_blue;
  std::ifstream json_file_yellow;
  std::ifstream json_file_green;
  std::ifstream json_file_red;

  //open json file and create json object
  json_file_blue.open(blue_button_task);
  json_file_yellow.open(yellow_button_task);
  json_file_green.open(green_button_task);
  json_file_red.open(red_button_task);

  if(json_file_blue && json_file_yellow && json_file_green && json_file_red)
    RCLCPP_INFO(this->get_logger(),"open all json files: %s",blue_button_task.c_str());
  else
    RCLCPP_ERROR(this->get_logger(),"failed to open one or more json files");
  json_data_blue = json::parse(json_file_blue);
  json_data_yellow = json::parse(json_file_yellow);
  json_data_green = json::parse(json_file_green);
  json_data_red = json::parse(json_file_red);
  RCLCPP_DEBUG(this->get_logger(),"json file detail: %s",json_data_blue.dump(3).c_str());
  RCLCPP_DEBUG(this->get_logger(),"json file detail: %s",json_data_yellow.dump(3).c_str());
  RCLCPP_DEBUG(this->get_logger(),"json file detail: %s",json_data_green.dump(3).c_str());
  RCLCPP_DEBUG(this->get_logger(),"json file detail: %s",json_data_red.dump(3).c_str());

  json_file_blue.close();
  json_file_yellow.close();
  json_file_green.close();
  json_file_red.close();

  RCLCPP_INFO(this->get_logger(),"json parse complete,close all the files");

  //init port name
  port_name_ = "/dev/ttybutton";
  connect(port_name_,B9600);

}

void rmf_button::submit_task_request(json& json_data){
  for (const auto& json_data_meta : json_data)
  {
    json json_msg;
    convert_task_to_request_msg(json_data_meta,json_msg);
    current_task_request.json_msg = json_msg.dump();

    // to add whitespace after echo character "," and ":"
    // for(std::string::iterator it = current_task_request.json_msg.begin();
    //   it != current_task_request.json_msg.end(); ++it)
    //  {
    //   cout<<"circle once"<<endl;
    //     if(*it == ',')
    //     {
    //       it = current_task_request.json_msg.insert(it+1,' ');
    //     }
    //     if(*it == ':')
    //     {          
    //       it = current_task_request.json_msg.insert(it+1,' ');
    //     }
    //  }

    current_task_request.request_id = request_id_const + std::to_string(request_id);
    request_id++;

    RCLCPP_INFO(this->get_logger(),"request id: %s",current_task_request.request_id.c_str());
      RCLCPP_DEBUG(this->get_logger(),"task request: %s",current_task_request.json_msg.c_str());
    tast_request_pub->publish(current_task_request);
    json_msg.clear();
    usleep(200000);

  }
}
void rmf_button::connect(std::string dev_name, uint32_t bouadrate) {
    RCLCPP_INFO(this->get_logger(),"connet the serial port:'%s'",dev_name.c_str());
    port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_name, bouadrate));
    
    if (port_->openPort() == 0) {
        // RCLCPP_INFO(this->get_logger(),"connet the serial port:1");
        // LimoDriver::readData();
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&rmf_button::readData, this)));
        read_data_thread_->detach();
         //read_data_thread_.join();
        // std::thread read_data_thread(std::bind(&LimoDriver::readData, this));
        // read_data_thread.join();
        // RCLCPP_INFO(this->get_logger(),"connet the serial port success :");
    }
    else {
            RCLCPP_ERROR(this->get_logger(),"Failed to open: '%s'",port_->getDevPath().c_str());
        // RCLCPP_ERROR("Failed to open %s", port_->getDevPath().c_str()); waring
        port_->closePort();
        exit(-1);
    }
}

  void rmf_button::pub_io_status(void)
  {
    GetAllDIStatus();

    io_status.io_status_1 = DI_Status.pin_1;
    io_status.io_status_2 = DI_Status.pin_2;
    io_status.io_status_3 = DI_Status.pin_3;
    io_status.io_status_4 = DI_Status.pin_4;

    iostatus_pub->publish(io_status);

    if(DI_Status.pin_1 == false && DI_Status.pin_2 == false 
              && DI_Status.pin_3 == false && DI_Status.pin_4 == false)
    {
      button_press_flag = false;
    }
    else
    {
      if(DI_Status.pin_1 == true && button_press_flag == false) //the button is pressed
      {
        button_press_flag = true;
        RCLCPP_INFO(this->get_logger(),"Button 1 (blue) is pressed! total send %d tasks",int(json_data_blue.size()));
        submit_task_request(json_data_blue);
      }
      if(DI_Status.pin_2 == true && button_press_flag == false) //the button is pressed
      {
        button_press_flag = true;
        RCLCPP_INFO(this->get_logger(),"Button 2 (yellow) is pressed! total send %d tasks",int(json_data_yellow.size()));
        submit_task_request(json_data_yellow);
      }
      if(DI_Status.pin_3 == true && button_press_flag == false) //the button is pressed
      {
        button_press_flag = true;
        RCLCPP_INFO(this->get_logger(),"Button 3 (green) is pressed! total send %d tasks",int(json_data_green.size()));
        submit_task_request(json_data_green);
      }
      if(DI_Status.pin_4 == true && button_press_flag == false) //the button is pressed
      {
        button_press_flag = true;
        RCLCPP_INFO(this->get_logger(),"Button 4 (red) is pressed! total send %d tasks",int(json_data_red.size()));
        submit_task_request(json_data_red);
      }
    }
  }
  void rmf_button::readData()
  {
        uint8_t rx_data = 0;
    // RCLCPP_INFO(this->get_logger(),"connet the serial port:2");
    while (rclcpp::ok()) {
        // RCLCPP_INFO(this->get_logger(),"connet the serial port:3");
        auto len = port_->readByte(&rx_data);
        // cout<<len<<"  begain read data  "<<(int)rx_data<<endl;

        if (len < 1)
            continue;
        // RCLCPP_INFO(this->get_logger(),"connet the serial port:'%s'",port_->readByte(&rx_data));
        // std::cout << "- the rx data: " << rx_data << std::endl;
        processRxData(rx_data);
        // RCLCPP_INFO(this->get_logger(),"connet the serial port:4");
        
    }
    // RCLCPP_INFO(this->get_logger(),"connet the serial port success readData:");

  }

  void rmf_button::processRxData(uint8_t data)
  {
    if(data == 0xFE)
    {
      //检测到帧头，buffer清零，标志位清零
      uart_buffer.clear();
      receive_flag = 0;

      uart_buffer.push_back(data);
      receive_flag++;

      return;
    }
    if((data == 0x02 || data == 0x01) && receive_flag == 1)
    {
      uart_buffer.push_back(data);
      receive_flag++;

      return;
    }
    if(receive_flag == 2 || receive_flag == 3 || receive_flag == 4 || receive_flag == 5)
    {
      uart_buffer.push_back(data);
      receive_flag++;
    }
    if(receive_flag == 6)
    {
      if(uart_buffer[1] == 0x01)
      {
        DO_Status.pin_1 = (uart_buffer[3] == 0x01);
        DO_Status.pin_2 = (uart_buffer[3] == 0x02);
        DO_Status.pin_3 = (uart_buffer[3] == 0x03);
        DO_Status.pin_4 = (uart_buffer[3] == 0x04);
      }
      if(uart_buffer[1] == 0x02)
      {
        DI_Status.pin_1 = (uart_buffer[3] == 0x01);
        DI_Status.pin_2 = (uart_buffer[3] == 0x02);
        DI_Status.pin_3 = (uart_buffer[3] == 0x03);
        DI_Status.pin_4 = (uart_buffer[3] == 0x04);
      }
      //循环结束
      // cout<<"DI Status:"<<DI_Status.pin_1<<DI_Status.pin_2<<DI_Status.pin_3<<DI_Status.pin_4<<endl;
      // cout<<"DO Status:"<<DO_Status.pin_1<<DO_Status.pin_2<<DO_Status.pin_3<<DO_Status.pin_4<<endl;
    }

  }

  void rmf_button::GetAllDOStatus(void)
  {
    uint8_t data_length = 8;
    uint8_t data[8];
    data[0] = 0xFE;
    data[1] = 0x01;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x04;
    data[6] = 0x29;
    data[7] = 0xc6;

    port_->writeData(data,data_length);
  }
  void rmf_button::GetAllDIStatus(void)
  {
    uint8_t data_length = 8;
    uint8_t write_date[8];
    write_date[0] = 0xFE;
    write_date[1] = 0x02;
    write_date[2] = 0x00;
    write_date[3] = 0x00;
    write_date[4] = 0x00;
    write_date[5] = 0x04;
    write_date[6] = 0x6D;
    write_date[7] = 0xc6;

    port_->writeData(write_date,data_length);
  }

  void rmf_button::convert_task_to_request_msg(const json& json_in,json& json_out)
  {


     json_out["type"] = "dispatch_task_request";
   // if(json_in["task_type"] == "Loop")
    //{
      json_out["request"]["description"]["places"] = {json_in["description"]["start_name"], json_in["description"]["finish_name"]};
      json_out["request"]["description"]["rounds"] = int(json_in["description"]["num_loops"]);
      json_out["request"]["category"] = "patrol";
    //}
    json_out["request"]["labels"] = {"rmf_demos.simple_api_server"};
    json_out["request"]["priority"] = {{"type", "binary"},{"value", 0}};

    json_out["request"]["unix_millis_earliest_start_time"] = 0;
    //json_out.push_back({"type", "dispatch_task_request"});

  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rmf_button> node = std::make_shared<rmf_button>("rmf_button");

  RCLCPP_INFO(node->get_logger(),"init complete!");
  rclcpp::spin(node);
  return 0;
}
