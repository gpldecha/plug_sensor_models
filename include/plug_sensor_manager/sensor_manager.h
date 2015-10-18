#ifndef PLUG_SENSOR_MANAGER_H_
#define PLUG_SENSOR_MANAGER_H_

///
/// \brief The Sensor_manager class
/// Managers a set of senor functions which populates a multivariate sensor vector Y
///
/// 1) Virtual sensor: computes probability of contact/no conact based on computing distance
///                    from the plug model to objects in the environement
///
/// 2) FT sensor: computes the probability of contact/no contact based on force measurements
///

#include <plug_sensor_models/plug_distance_model.h>
#include <plug_sensor_models/distance_model.h>
#include <plug_sensor_models/force_iid_model.h>

#include <armadillo>
#include <memory>

#include <wrapobject/wrapobject.h>

namespace psm{

typedef enum {NONE,
              SIMPLE_CONTACT_DIST,
              FOUR_CONTACT_DIST,
              THREE_PIN_DIST,
              FORCE_IID
             } type_sensor;

class Sensor_manager{

public:

    typedef std::shared_ptr<psm::Plug_distance_model>       Sptr_dist;
    typedef std::shared_ptr<psm::Contact_distance_model>    Sptr_cdist;
    typedef std::shared_ptr<psm::Three_pin_distance_model>  Sptr_three_dist;
    typedef std::shared_ptr<psm::Force_iid_model>           Sptr_fii;

public:

    Sensor_manager(wobj::WrapObject& wrapped_objects);

    void initialise(type_sensor t_sensor);

    void update(arma::colvec& Y, const arma::colvec3& pos, const arma::mat33& Rot, const arma::fcolvec& force);

  /*  void update_force(arma::colvec& Y,const arma::fcolvec3& force);

    void update_dist(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);*/

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
