#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <netft_rdt_driver/ft_listener.h>
#include <armadillo>


#include "optitrack_rviz/filter.h"

#include "peg_sensor/classifier/peg_classifier.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"





/**
*       ===  Force Torque sensor node ===
*
*      o Node subscribes to the force-torque sensor, filters the noise and removes the bias
*        and republished the filtered and bias free force-torque signal.
*
*      o Publishes also a classficiation result (contact/no contact, left contact, right contact)
*        based on the filtered-force torque sensor
*
**/

int main(int argc,char** argv){


    // -------------- Get node input paramters --------------

    std::map<std::string,std::string> input;
    input["-listen_topic"]      = "";
    input["-publish_topic"]     = "";
    input["-fixed_frame"]       = "/world_frame";
    input["-peg_link_name"]     = "";
    input["-rate"]              = "100";
    input["-print"]             = "false";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string sensor_publish_topic  = input["-y_topic"];
    std::string sensor_listener_topic = input["-ft_topic"];
    std::string fixed_frame           = input["-fixed_frame"];
    std::string path_sensor_model     = input["-path_sensor_model"];
    std::string peg_link_name         = input["-peg_link_name"];

    // -------------- Initialise node --------------

    ros::init(argc, argv, "peg_sensor_classifier");
    ros::NodeHandle nh;

    Peg_world_wrapper peg_world_wrapper(nh,path_sensor_model,fixed_frame);
    wobj::WrapObject& wrapped_objects = peg_world_wrapper.get_wrapped_objects();

    psm::Sensor_manager sensor_manager(wrapped_objects);
    psm::Peg_sensor_clasifier(nh,
                              fixed_frame,
                              peg_link_name,
                              sensor_manager,
                              sensor_listener_topic,
                              sensor_publish_topic);
    ros::Rate rate(rate_hz);

    while(nh.ok()){


        peg_world_wrapper.update();




        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
