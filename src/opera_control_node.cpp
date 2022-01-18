#include "opera_control/bt_nodes.h"

BT::NodeStatus MovePlat1::tick() 
{
    Optional<int> msg = getInput<int>("position");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_control::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    //TODO check response
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat2::tick() 
{
    Optional<int> msg = getInput<int>("position");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_control::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    //TODO check response
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat3::tick() 
{
    Optional<int> msg = getInput<int>("position");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_control::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    //TODO check response
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::NodeStatus MovePlat4::tick() 
{
    Optional<int> msg = getInput<int>("position");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [position]: ", 
                                msg.error() );
    }

    _halt_requested.store(false);
    pkg_rp_control::MovePlatSrv srv;
    srv.request.position_command = msg.value();
    MoveService.call(srv);
    //TODO check response
    _position = srv.response.position_final;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "opera_control_node");
    ros::NodeHandle nh("~");

    std::string tree_path;
    if (!nh.getParam("tree_path", tree_path))
    {
        ROS_ERROR("Param tree_path not found");
        return 0;
    }
    std::cout << "0 ";
    ROS_INFO_STREAM("tree_path: " << tree_path);


    ros::AsyncSpinner spinner(2);
    spinner.start();

    BehaviorTreeFactory factory;


    SystemCheck checker(&nh);
    factory.registerSimpleAction("WaitAndCheckStatus", 
                                 std::bind(&SystemCheck::WaitAndCheckStatus, &checker));


    factory.registerNodeType<MovePlat1>("MovePlat1");
    factory.registerNodeType<MovePlat2>("MovePlat2");
    factory.registerNodeType<MovePlat3>("MovePlat3");
    factory.registerNodeType<MovePlat4>("MovePlat4");

    
    auto tree = factory.createTreeFromFile(tree_path);

    PublisherZMQ publisher_zmq(tree);


    while(ros::ok())
    {
        auto asd = tree.tickRoot();
        ros::Duration(.3).sleep();
        
    }
    

    ros::waitForShutdown();

    return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/