#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <pkg_rp_msgs/MovePlatSrv.h>
#include <iostream>
#include <fstream>
#include <std_srvs/Trigger.h>

// Example of custom SyncActionNode (synchronous action)
// without ports.

using namespace BT;

float sleep_time = 2.0;

class SystemCheck
{
  public:
    SystemCheck(ros::NodeHandle *nh_ext)
    {
      _nh = *nh_ext;
      if (!_nh.getParam("rp1_status_topic", _rp1_status_topic)) {ROS_ERROR("Param rp1_status_topic not found, set to /rp1/status");}
      if (!_nh.getParam("rp2_status_topic", _rp2_status_topic)) {ROS_ERROR("Param rp2_status_topic not found, set to /rp2/status");}
      if (!_nh.getParam("rp3_status_topic", _rp3_status_topic)) {ROS_ERROR("Param rp3_status_topic not found, set to /rp3/status");}
      if (!_nh.getParam("rp4_status_topic", _rp4_status_topic)) {ROS_ERROR("Param rp4_status_topic not found, set to /rp4/status");}
      Stop_service   = _nh.advertiseService("Stop_service" , &SystemCheck::RequestStop , this);
      Start_service  = _nh.advertiseService("Start_service", &SystemCheck::Requeststart, this);

    };

    BT::NodeStatus WaitAndCheckStatus()
    {
      _rp1_status = ros::topic::waitForMessage<std_msgs::Int8>(_rp1_status_topic, _nh, ros::Duration(2));
      _rp2_status = ros::topic::waitForMessage<std_msgs::Int8>(_rp2_status_topic, _nh, ros::Duration(2));
      _rp3_status = ros::topic::waitForMessage<std_msgs::Int8>(_rp3_status_topic, _nh, ros::Duration(2));
      _rp4_status = ros::topic::waitForMessage<std_msgs::Int8>(_rp4_status_topic, _nh, ros::Duration(2));
      if(_rp1_status == nullptr){ROS_FATAL("No status from rp1");return BT::NodeStatus::FAILURE;}
      if(_rp2_status == nullptr){ROS_FATAL("No status from rp2");return BT::NodeStatus::FAILURE;}
      if(_rp3_status == nullptr){ROS_FATAL("No status from rp3");return BT::NodeStatus::FAILURE;}
      if(_rp4_status == nullptr){ROS_FATAL("No status from rp4");return BT::NodeStatus::FAILURE;}
      std::cout << "[ Status:  " << int((*_rp1_status).data) << " " << int((*_rp2_status).data) << " " << int((*_rp3_status).data) << " " << int((*_rp4_status).data) << " ]" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    bool isStopping(){return _stop_request;}
    bool isStarting(){return _start_request;}

    bool RequestStop(std_srvs::Trigger::Request  &req,
              std_srvs::Trigger::Response &res);

    bool Requeststart(std_srvs::Trigger::Request  &req,
              std_srvs::Trigger::Response &res);
  
  private:
    ros::NodeHandle _nh;
    std_msgs::Int8ConstPtr _rp1_status = nullptr;
    std_msgs::Int8ConstPtr _rp2_status = nullptr;
    std_msgs::Int8ConstPtr _rp3_status = nullptr;
    std_msgs::Int8ConstPtr _rp4_status = nullptr;
    std::string _rp1_status_topic = "";
    std::string _rp2_status_topic = "";
    std::string _rp3_status_topic = "";
    std::string _rp4_status_topic = "";

    ros::ServiceServer Start_service;
    ros::ServiceServer  Stop_service;

    bool _stop_request  = false;
    bool _start_request = false;

    std::vector<uint8_t> _status {0,0,0,0};
    std::vector<std::vector<uint8_t>> _configurations {{0,1,0,0},
                                                       {0,1,0,1},
                                                       {1,1,0,4},
                                                       {1,1,1,3},
                                                       {1,1,0,4},
                                                       {0,1,0,1},
                                                       {0,1,0,0},
                                                       {0,0,0,0}};
                                                       

};


class MovePlat1 : public AsyncActionNode
{
  public:
    MovePlat1(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      MoveService = nh.serviceClient<pkg_rp_msgs::MovePlatSrv> ("/rp_control_rp1/MovePlat_rp1");
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("position") };
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
        _halt_requested.store(true);
    }

    int get_pos(){return _position;}

  friend class SystemCheck;

  private:
    ros::ServiceClient MoveService;
    std::atomic_bool _halt_requested;
    int _position = 0;

};


class MovePlat2 : public AsyncActionNode
{
  public:
    MovePlat2(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      MoveService = nh.serviceClient<pkg_rp_msgs::MovePlatSrv> ("/rp_control_rp2/MovePlat_rp2");
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("position") };
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
        _halt_requested.store(true);
    }

    int get_pos(){return _position;}

    friend class SystemCheck;

  private:
    ros::ServiceClient MoveService;
    std::atomic_bool _halt_requested;
    int _position = 0;
};


class MovePlat3 : public AsyncActionNode
{
  public:
    MovePlat3(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      MoveService = nh.serviceClient<pkg_rp_msgs::MovePlatSrv> ("/rp_control_rp3/MovePlat_rp3");
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("position") };
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
        _halt_requested.store(true);
    }

    int get_pos(){return _position;}

    friend class SystemCheck;

  private:
    ros::ServiceClient MoveService;
    std::atomic_bool _halt_requested;
    int _position = 0;
};


class MovePlat4 : public AsyncActionNode
{
  public:
    MovePlat4(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      MoveService = nh.serviceClient<pkg_rp_msgs::MovePlatSrv> ("/rp_control_rp4/MovePlat_rp4");
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("position") };
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
        _halt_requested.store(true);
    }

    int get_pos(){return _position;}

    friend class SystemCheck;

  private:
    ros::ServiceClient MoveService;
    std::atomic_bool _halt_requested;
    int _position = 0;
};

