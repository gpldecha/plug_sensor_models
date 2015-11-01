#include "peg_sensor/classifier/force_iid_model.h"

namespace psm {

Force_iid_model::Force_iid_model(vector_type type):
type(type){
    beta = 1;

    fenum2str[contact]  = "contact";
    fenum2str[up]       = "up";
    fenum2str[down]     = "down";
    fenum2str[left]     = "left";
    fenum2str[right]    = "right";
    fenum2str[push]     = "push";

    limits.resize(6);
    var.resize(6);

    limits[contact] = limit(0.9,1.5);
    limits[up]      = limit(0.5,1.2);
    limits[down]    = limit(-1.2,-0.5);
    limits[left]    = limit(-1.2,-0.5);
    limits[right]   = limit(0.5,1.2);
    limits[push]    = limit(-3.5,-1.5);

    for(std::size_t i = 0; i < limits.size();i++){
        var[i]   =    std::pow((limits[i].lower - limits[i].upper)/3,2);
    }


}

void Force_iid_model::update(arma::colvec& Y,const arma::fcolvec3& force, const arma::fcolvec3& torque){

    features(contact) = gaussian_step_function_positive(arma::norm(force),0.9,1,beta);

    features(up)     = gaussian_step_function_positive(force(0),limits[contact].lower,limits[contact].upper,var[contact]);   //+

    features(down)   = gaussian_step_function_negative(force(0),limits[down].lower,limits[down].upper,var[down]);       //-
    features(left)   = gaussian_step_function_negative(force(1),limits[left].lower,limits[left].upper,var[left]);       // -

    features(right)  = gaussian_step_function_positive(force(1),limits[right].lower,limits[right].upper,var[right]);    // +

    features(push)   = gaussian_step_function_negative(force(2),limits[push].lower,limits[push].upper,var[push]);       //-

    //contact,up,down,left,right,push

    features_simple(0) = features(contact);
    features_simple(1) = arma::max(features(arma::span(1,4)));


    if(type == FULL){
        Y.resize(6);
        Y(contact)  = features(contact);
        Y(up)       = features(up);
        Y(down)     = features(down);
        Y(left)     = features(left);
        Y(right)    = features(right);
        Y(push)     = features(push);
    }else{
        Y.resize(2);
        Y(0)        = features_simple(0);
        Y(1)        = features_simple(1);
    }
}

void Force_iid_model::print() const{

    if(type == FULL){
        std::cout<< "c: "  << features(contact) <<
                    " u: " << features(up)      <<
                    " d: " << features(down)    <<
                    " l: " << features(left)    <<
                    " r: " << features(right)   <<
                    " p: " << features(push)    << std::endl;
    }else{
        std::cout<< "c: " << features_simple(0) << " e: " << features_simple(1) << std::endl;
    }
}




}
