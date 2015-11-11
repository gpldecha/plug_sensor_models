#include "peg_sensor/classifier/peg_classifier.h"
#include <optitrack_rviz/type_conversion.h>

namespace psm{


Peg_sensor_clasifier::Peg_sensor_clasifier(ros::NodeHandle&   nh,
                                           const std::string& fixed_frame,
                                           const std::string& peg_link_name,
                                           Sensor_manager&    sensor_manager,
                                           const std::string& ft_topic_name,
                                           const std::string& y_topic_name):
 peg_pos_listener(fixed_frame,peg_link_name),
  sensor_manager(sensor_manager)
{


    ft_sub = nh.subscribe<geometry_msgs::WrenchStamped>(ft_topic_name,1,&Peg_sensor_clasifier::ft_callback,this);

    y_publisher = nh.advertise<std_msgs::Float32MultiArray>(y_topic_name,10);

}


void Peg_sensor_clasifier::ft_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){

    ft_force(0) = msg->wrench.force.x;
    ft_force(1) = msg->wrench.force.y;
    ft_force(2) = msg->wrench.force.z;

    ft_torque(0) = msg->wrench.torque.x;
    ft_torque(1) = msg->wrench.torque.y;
    ft_torque(2) = msg->wrench.torque.z;

}

void Peg_sensor_clasifier::update(){

    peg_pos_listener.update(peg_origin_tf,peg_orient_tf);
    opti_rviz::type_conv::tf2mat(peg_orient_tf,peg_orient);
    opti_rviz::type_conv::tf2vec(peg_origin_tf,peg_origin);

    sensor_manager.update_peg(Y,peg_origin,peg_orient);

    y_msg.data.resize(Y.n_elem);
    for(std::size_t i = 0; i < Y.n_elem;i++){
        y_msg.data[i] = Y(i);
    }

    y_publisher.publish(y_msg);

}

}

