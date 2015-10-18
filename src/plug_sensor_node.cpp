#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <netft_rdt_driver/ft_listener.h>
#include <armadillo>
#include <plug_sensor_models/force_iid_model.h>
#include "optitrack_rviz/filter.h"



int main(int argc,char** argv){

    std::map<std::string,std::string> input;
    input["-listen_topic"]     = "";
    input["-publish_topic"]    = "";
    input["-rate"]             = "100";

   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv, "plug_sensor_model");
    ros::NodeHandle node;
    float      r  = boost::lexical_cast<float>(input["-rate"]);
    ros::Rate rate(r);

    psm::Force_iid_model        plug_sensor_model;
    psm::Plug_sensor_publisher  plug_sensor_publisher(node,input["-publish_topic"],6);

    netft::Ft_listener ft_listener(node,input["-listen_topic"]);
    arma::fcolvec3 force;
    force(0) = ft_listener.current_msg.wrench.force.x;
    force(1) = ft_listener.current_msg.wrench.force.y;
    force(2) = ft_listener.current_msg.wrench.force.z;

    ros::Publisher pub_ff = node.advertise<std_msgs::Float32MultiArray>("ff",10);
    ros::Publisher pub_ff_dt = node.advertise<std_msgs::Float32MultiArray>("ff_dt",10);

    std_msgs::Float32MultiArray filtered_force_msg;
    filtered_force_msg.data.resize(3);


    opti_rviz::Kalman kalman(1/r,0.1,1);
    tf::Vector3 tf_force(force(0),force(1),force(2));
    tf::Vector3 tf_force_tmp;
    kalman.init(tf_force);
    tf::Vector3 tf_force_dt;


    while(node.ok()){

        force(0) = ft_listener.current_msg.wrench.force.x;
        force(1) = ft_listener.current_msg.wrench.force.y;
        force(2) = ft_listener.current_msg.wrench.force.z;

        tf_force[0] = force(0);
        tf_force[1] = force(1);
        tf_force[2] = force(2);

        kalman.update(tf_force);

        tf_force_dt = tf_force - tf_force_tmp;

        filtered_force_msg.data[0] = tf_force[0];
        filtered_force_msg.data[1] = tf_force[1];
        filtered_force_msg.data[2] = tf_force[2];

        pub_ff.publish(filtered_force_msg);


        filtered_force_msg.data[0] = tf_force_dt[0];
        filtered_force_msg.data[1] = tf_force_dt[1];
        filtered_force_msg.data[2] = tf_force_dt[2];

        pub_ff_dt.publish(filtered_force_msg);



        //plug_sensor_model.update(force);
       // plug_sensor_publisher.publish(plug_sensor_model.features);

        tf_force_tmp = tf_force;

        ros::spinOnce();
        rate.sleep();
   }

    return 0;
}
