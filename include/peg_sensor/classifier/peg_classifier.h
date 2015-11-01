#ifndef PEG_SENSOR_CLASSIFIER_H_
#define PEG_SENSOR_CLASSIFIER_H_

#include "peg_sensor_manager/sensor_manager.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"
#include <geometry_msgs/WrenchStamped.h>
#include <optitrack_rviz/listener.h>

#include <armadillo>

/**
 *    === Peg sensor classifier ===
 *
 *  Computes a sensor output Y_t (contact/no contact, etc..) from one of two methods;
 *  1) Y_t is computed from virtual environment
 *  2) Y_t is computed from the force-torque sensor
 *
 *  The user can switch between either of the two methods through making a ros service
 *  call.
 *
 *  The Y_t multivariate sensing vector can then be tipically used by a filter.
 *
 */


namespace psm{

class Peg_sensor_clasifier{

public:

    Peg_sensor_clasifier(ros::NodeHandle& nh,
                         const std::string& fixed_frame,
                         const std::string& peg_link_name,
                         Sensor_manager& sensor_manager,
                         const std::string& ft_topic_name,
                         const std::string& y_topic_name);



    /**
     * @brief update    : computes new sensor measurement Y_t
     *                    and publishes it.
     */
    void update();

private:

    void ft_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

private:

    opti_rviz::Listener peg_pos_listener;

    Sensor_manager&     sensor_manager;
    ros::Subscriber     ft_sub;

    arma::colvec3       peg_origin;
    arma::mat33         peg_orient;
    arma::fcolvec3      ft_force;
    arma::fcolvec3      ft_torque;
    arma::colvec        Y;

    tf::Vector3         peg_origin_tf;
    tf::Matrix3x3       peg_orient_tf;

    std_msgs::Float32MultiArray y_msg;
    ros::Publisher              y_publisher;





};

}


#endif
