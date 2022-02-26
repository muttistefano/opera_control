#include "opera_control/bt_nodes.h"
#include <time.h>

BT::NodeStatus MovePlat1::tick() 
{
    Optional<int> msg = getInput<int>("position");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }
    _halt_requested.store(false);
    pkg_rp_msgs::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat2::tick() 
{
    Optional<int> msg = getInput<int>("position");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_msgs::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat3::tick() 
{
    Optional<int> msg = getInput<int>("position");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_msgs::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat4::tick() 
{
    Optional<int> msg = getInput<int>("position");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_msgs::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

bool SystemCheck::RequestStop(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    ROS_ERROR("Stop requested from service");
    this->_stop_request  = true;
    this->_start_request = false;
    return true;
}

bool SystemCheck::Requeststart(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    ROS_ERROR("Start requested");
    this->_stop_request  = false;
    this->_start_request = true;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "opera_control_node");
    ros::NodeHandle nh("~");

    std::string tree_path;
    std::string tree_intro;

    srand(time(nullptr));

    if (!nh.getParam("tree_path", tree_path))
    {
        ROS_ERROR("Param tree_path not found");
        return 0;
    }

    
    ROS_INFO_STREAM("tree_path: "  << tree_path);

    ros::Publisher offer_pub = nh.advertise<std_msgs::Bool>("offer",1);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    BehaviorTreeFactory factory;

    SystemCheck checker(&nh);
    
    factory.registerSimpleAction("WaitAndCheckStatus", std::bind(&SystemCheck::WaitAndCheckStatus, &checker));


    factory.registerNodeType<MovePlat1>("MovePlat1");
    factory.registerNodeType<MovePlat2>("MovePlat2");
    factory.registerNodeType<MovePlat3>("MovePlat3");
    factory.registerNodeType<MovePlat4>("MovePlat4");

    
    
    auto tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(tree_path));

    // PublisherZMQ publisher_zmq(tree);

    ros::Duration(5).sleep();


    ROS_INFO("STARTING\n");

    int cycle = 0;

    while(ros::ok())
    {

        auto ret_tree = tree->tickRoot();
        ros::Duration(.5).sleep();

        double rwait = 20;

        if ( ret_tree == BT::NodeStatus::SUCCESS)
        {
            if(checker.isStopping())
            {
              ROS_FATAL("System stopped \n");  
              while(!checker.isStarting() && ros::ok())
              {
                  ROS_WARN("SYSTEM IDLE \n");
                  std_msgs::Bool b_ms;
                  b_ms.data = true;
                  offer_pub.publish(b_ms);
                  ros::Duration(2).sleep();
              }
            }
            ROS_WARN("Cycle finished \n");
            ros::Duration(300).sleep();
        }
        else if ( ret_tree == BT::NodeStatus::FAILURE)
        {
            checker.SendStop();
            while(!checker.isStarting() && ros::ok())
            {
                ROS_WARN("SYSTEM IDLE \n");
                std_msgs::Bool b_ms;
                b_ms.data = true;
                offer_pub.publish(b_ms);
                ros::Duration(2).sleep();
            }
            cycle = 0;
        }
        rwait = fRand(3,25);
        ros::Duration(rwait).sleep();
        cycle++;

    }
    

    ros::waitForShutdown();

    return 0;
}
