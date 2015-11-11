#ifndef PEG_SENSOR_MANAGER_H_
#define PEG_SENSOR_MANAGER_H_

///
/// \brief The Sensor_manager class
/// Managers a set of senor functions which populates a multivariate sensor vector Y
///
/// 1) Virtual sensor: computes probability of contact/no conact based on computing distance
///                    from the plug model to objects in the environement
///
/// 2) FT sensor: computes the probability of contact/no contact based on force measurements
///

#include "peg_sensor/peg_sensor_model/peg_distance_model.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "peg_sensor/classifier/force_iid_model.h"
#include <std_msgs/Float32MultiArray.h>
#include <peg_sensor/String_cmd.h>


#include <armadillo>
#include <memory>

#include "wrapobject.h"

namespace psm{

typedef enum {NONE,
              SIMPLE_CONTACT_DIST,
              THREE_PIN_DIST,
              FORCE_IID
             } type_sensor;



class Sensor_manager{

public:

    typedef std::shared_ptr<psm::Contact_distance_model>    Sptr_cdist;
    //typedef std::shared_ptr<psm::Three_pin_distance_model>  Sptr_three_dist;
    typedef std::shared_ptr<psm::Force_iid_model>           Sptr_fii;

public:

    Sensor_manager(ros::NodeHandle& nh,wobj::WrapObject& wrapped_objects,obj::Socket_one& socket_one);


    // called for one point at the time
   /* void update(arma::colvec& Y,
                const arma::colvec3& pos,
                const arma::mat33& Rot,
                const arma::fcolvec3& force,
                const arma::fcolvec3& torque);*/


    /**
     * @brief update_peg        : computes actual sensation Y from the peg end-effector.
     *                            This function would tipically be used in the peg_sensor node
     *                            which publishes the sensations Y felt by the peg. In our case
     *                            Y is a probability distribution over a set of discrete features.
     */
    void update_peg(arma::colvec& Y,const arma::colvec3& pos, const arma::mat33& Ro);

    /**
     * @brief update_particles  : computes expected sensation hY for a set of hypothetical
     *                            positions of the end-effector. This function would tipically
     *                            be used on the particle filters side to compute hypothetical
     *                            sensations, hY, which are then used to compute the likelihood
     *                            of each particle.
     */
    void update_particles(arma::mat& Y,const arma::mat& points, const arma::mat33& Rot);


  //  void init_visualise(ros::NodeHandle& node);

    // void visualise();

//    void print(arma::colvec& Y);

private:

    void initialise();

    bool sensor_manager_callback(peg_sensor::String_cmd::Request& req, peg_sensor::String_cmd::Response& res);


public:

    type_sensor             t_sensor;


private:

    Sptr_cdist              sptr_cdist;
    //Sptr_three_dist         sptr_three_dist;
    Sptr_fii                ptr_sensor_force_idd;

    wobj::WrapObject&       wrapped_objects;
    obj::Socket_one&        socket_one;


    ros::ServiceServer      service_server;



};

}



#endif
