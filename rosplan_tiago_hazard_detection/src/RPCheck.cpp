//
// Created by robot on 3/29/19.
//

#include "RPCheck.h"



/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPCheck::RPCheck(ros::NodeHandle &nh) {
        // perform setup
    }

    /* action dispatch callback */
    bool RPCheck::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;

        // log available parameters
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            ROS_INFO("%s <----> %s", it->key.c_str(), it->value.c_str());
        }

        // Get the actual values by calling the service


        ROS_INFO(msg.get()->parameters.back().value.c_str());
        //Client client("check", true); // true -> don't need ros::spin()
//        ROS_INFO("CLIENT: CHECK: Waiting for sever");
        client.waitForServer();
        rosplan_tiago_hazard_detection::CheckGoal goal;

        // Fill in goal here
        goal.blind_goal = 500;

//        ROS_INFO("CLIENT: CHECK: I will send goal now");
        client.sendGoal(goal);

//        ROS_INFO("CLIENT: CHECK: I will wait for result now");
        client.waitForResult(ros::Duration(10.0));

//        ROS_INFO("CLIENT: CHECK: I received result and it is:");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        }

//        ROS_INFO("CLIENT: CHECK: Current State: %s\n", client.getState().toString().c_str());
//        ROS_INFO("CLIENT: CHECK: Result: %f\n", client.getResult().get()->is_undocked);
        // complete the action
        ROS_INFO("KCL: (%s) CHECK Action completing.\n", msg->name.c_str());
        return true;


    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_check_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // Create service client here
//    ros::ServiceClient client = nh.serviceClient<>("location_srv")


    // create PDDL action subscriber
    KCL_rosplan::RPCheck rpti(nh);

    rpti.runActionInterface();

    return 0;
}