#ifndef PLUG_SENSOR_H_
#define PLUG_SENSOR_H_

/**

**/


#include "wrapobject.h"
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <tf/transform_listener.h>
#include <optitrack_rviz/listener.h>
#include <visualise/vis_vector.h>

#include <armadillo>



typedef enum contact_type{
    SURFACE=0,EDGE=1,NONE=2
} contact_type;


class peg_enum_convert{

public:

    static std::string contact_type_2_string(contact_type contact_t){
    switch(contact_t){
    case SURFACE:
    {
        return "SURFACE";
    }
    case EDGE:
    {
        return "EDGE";
    }
    case NONE:
    {
        return "NONE";
    }
    default:
    {
        return "NOT DEFINED";
    }
    }
}

};

class Contact_points{

public:

    Contact_points(contact_type contact_t=contact_type::NONE):
        contact_t(contact_t)
    {
        index     = 0;
        distance  = -1;
        closest_point.zeros();
    }


    void print() const{
        std::cout<< "=== " << peg_enum_convert::contact_type_2_string(contact_t) << " ===" << std::endl;
        std::cout<< " index:      " << index << std::endl;
        std::cout<< " distance:   " << distance << std::endl;
        std::cout<< " closest pt: " << closest_point(0) << " " << closest_point(1) << " " << closest_point(2) << std::endl;
        std::cout<<std::endl;
    }

    contact_type    contact_t;
    std::size_t     index;
    float           distance;
    arma::fcolvec3  closest_point;


};

class Peg_sensor_model{


public:

    Peg_sensor_model(const std::string& path_to_peg_model, const std::string &fixed_frame,
                     wobj::WrapObject &wrap_object);

    void update();

    const std::vector<tf::Vector3>& get_model();

    const std::vector<tf::Vector3>& get_closet_point();

    const std::vector<opti_rviz::Arrow>& get_arrows();

private:

    void get_distance_features();

    void update_model(const tf::Vector3& T, const tf::Matrix3x3& R);


private:


    wobj::WrapObject&               wrapped_world;
    std::vector<tf::Vector3>        model,model_TF;
    std::vector<Contact_points>     contact_info;
    opti_rviz::Listener             tf_listener;
    tf::Vector3                     position;
    tf::Matrix3x3                   orientation;
    arma::fcolvec3                  tmp_vec3f;
    tf::Vector3                     tmp_Vec3;

    std::vector<opti_rviz::Arrow>   arrows;
    std::vector<tf::Vector3>        closest_points;

    float                           min_distance_edge;
    float                           min_distance_surface;
    float                           current_distance_surface;
    float                           current_distance_edge;

};

#endif
