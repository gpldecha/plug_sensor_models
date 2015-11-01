#ifndef PEG_FORCE_IID_MODEL_H_
#define PEG_FORCE_IID_MODEL_H_

#include "peg_sensor/classifier/base_peg_classifier.h"

#include <ros/ros.h>
#include <armadillo>
#include <map>

namespace psm{

typedef enum {SIMPLE,FULL} vector_type;
typedef enum {contact=0,up=1,down=2,left=3,right=4,push=5} feature_type;



class Force_iid_model : public Base_peg_classifier {

public:

    struct limit{
        limit(){}
        limit(float lower,float upper):upper(upper),lower(lower){}
        float upper;
        float lower;
    };

public:

    Force_iid_model(vector_type type = FULL);

     virtual void update(arma::colvec& Y,const arma::fcolvec3& force, const arma::fcolvec3& torque);

    void print() const;

private:

    inline float gaussian_step_function(const float x, float min_x, float max_x, float beta){
        if(x < min_x){
            return 0;
        }else if(x > max_x){
            return 1;
        }else{
            return exp(-beta * (x - max_x)*(x - max_x));
        }
    }

    inline float gaussian_step_function_positive(const float x,float lower, float upper,float var){
        if(x < lower){
            return 0;
        }else if(x > upper){
            return 1;
        }else{
            return exp(-0.5 * (1/var) * (x - upper)*(x - upper));
        }
    }


    inline float gaussian_step_function_negative(const float x,float lower, float upper,float var){
        if(x < lower){
            return 1;
        }else if(x > upper){
            return 0;
        }else{
            return exp(-0.5 * (1/var) * (x - lower)*(x - lower));
        }
    }

public:

    arma::fcolvec2                      features_simple;
    arma::fcolvec6                      features;
    vector_type                         type;

private:

    std::map<feature_type,std::string>  fenum2str;
    std::vector<limit>                  limits;
    std::vector<float>                  var;
    float                               beta;

};



}


#endif
