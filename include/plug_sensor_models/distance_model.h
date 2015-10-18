#ifndef DISTANCE_MODEL_H_
#define DISTANCE_MODEL_H_

#include <plug_sensor_models/plug_distance_model.h>
#include <wrapobject/wrapobject.h>
#include <visualise/vis_vector.h>

namespace psm{


class Three_pin_distance_model : public Plug_distance_model{
public:

    Three_pin_distance_model(wobj::WrapObject& wrap_object);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();



private:

    Distance_features                       distance_features;
    arma::fcolvec3                          tmp;
    std::shared_ptr<opti_rviz::Vis_vectors> ptr_vis_vectors;
    std::vector<opti_rviz::Arrow>           dir_vectors;
    std::vector<tf::Vector3>                colors;


};


class Contact_distance_model : public Plug_distance_model{

public:

    typedef enum {C_SURF,C_EDGE} contact_types;

public:

    Contact_distance_model(wobj::WrapObject& wrap_object);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();

protected:

    void get_distances();

private:

    double                              min_half_one_div_var;

protected:

    Distance_features                  distance_features;

    std::vector<opti_rviz::Arrow>           dir_vectors;
    std::vector<tf::Vector3>                colors;
    std::shared_ptr<opti_rviz::Vis_vectors> ptr_vis_vectors;
    std::shared_ptr<opti_rviz::Vis_points>  ptr_proj_points;

    arma::fmat                              proj_points;


    arma::fcolvec3                      tmp;

    float                               min_distance_edge;
    float                               min_distance_surface;
    float                               current_distance_surface;
    float                               current_distance_edge;

    std::size_t                         index_closest_model_surf;
    std::size_t                         index_closest_model_edge;

    arma::fcolvec3                      direction_surf;
    arma::fcolvec3                      direction_edge;

    arma::fcolvec3                      closet_point_proj_surf;
    arma::fcolvec3                      closest_model_surf;

    arma::fcolvec3                      closet_point_proj_edge;
    arma::fcolvec3                      closet_model_edge;


};


class Four_contact_distance_model : public Contact_distance_model {

public:

    Four_contact_distance_model(wobj::WrapObject &wrap_object);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

public:

     arma::fcolvec3                     edge_dir_norm;

     float                              degree_var;
     float                              dist_var;
     float                              three_std;
     float                              norm_direction;
     std::vector<arma::fcolvec3>        directions;

};



}

#endif
