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


#include <armadillo>
#include <memory>

#include "wrapobject.h"

namespace psm{

typedef enum {NONE,
              SIMPLE_CONTACT_DIST,
              FOUR_CONTACT_DIST,
              THREE_PIN_DIST,
              FORCE_IID
             } type_sensor;

class Sensor_manager{

public:

    typedef std::shared_ptr<psm::Peg_distance_model>        Sptr_dist;
    typedef std::shared_ptr<psm::Contact_distance_model>    Sptr_cdist;
    typedef std::shared_ptr<psm::Three_pin_distance_model>  Sptr_three_dist;
    typedef std::shared_ptr<psm::Force_iid_model>           Sptr_fii;

public:

    Sensor_manager(wobj::WrapObject& wrapped_objects);

    void initialise(type_sensor t_sensor);

    void update(arma::colvec& Y,
                const arma::colvec3& pos,
                const arma::mat33& Rot,
                const arma::fcolvec3& force,
                const arma::fcolvec3& torque);


    void init_visualise(ros::NodeHandle& node);

    void visualise();

    void print(arma::colvec& Y);

public:

    Sptr_dist               sptr_dist;
    Sptr_fii                ptr_sensor_force_idd;
    type_sensor             t_sensor;
    wobj::WrapObject&       wrapped_objects;


private:




};

}



#endif
