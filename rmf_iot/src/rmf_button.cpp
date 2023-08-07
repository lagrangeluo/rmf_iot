#include <cstdio>
#include <rmf_iot/rmf_button.hpp>

using namespace AgileX;
using namespace std;


  rmf_button::rmf_button(string node_name):Node(node_name)
{
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

    json_msg = "{\"type\": \"dispatch_task_request\", \"request\": {\"priority\": {\"type\": \"binary\", \"value\": 0}, \"labels\": [\"rmf_demos.simple_api_server\"], \"description\": {\"pickup\": {\"place\": \"right\", \"handler\": \"pickup\", \"payload\": []}, \"dropoff\": {\"place\": \"left\", \"handler\": \"dropoff\", \"payload\": []}}, \"category\": \"delivery\", \"unix_millis_earliest_start_time\": 0}}";
    request_id_const = "demos_9180aa2d-66bc-4d61-8856-";
    request_id = "8e2525d77630";

  //init port name
  port_name_ = "/dev/ttybutton";
  connect(port_name_,B9600);

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

    if(DI_Status.pin_1 == true && button_press_flag == false) //the button is pressed
    {
      button_press_flag = true;
      current_task_request.json_msg = json_msg;
      current_task_request.request_id = request_id_const + request_id;
      //request_id = (std::string)(request_id + 1);

      tast_request_pub->publish(current_task_request);
    }
    else if(DI_Status.pin_1 == false)
    {
      button_press_flag = false;
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


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rmf_button> node = std::make_shared<rmf_button>("rmf_button");

  cout << "init complete!"<< endl;
  rclcpp::spin(node);
  return 0;
}
