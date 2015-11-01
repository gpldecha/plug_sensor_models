#include "peg_sensor/classifier/gmm_model.h"

namespace psm {

GMM_model::GMM_model(const std::string &name):
    name(name)
{
}


GMM_feature_model::GMM_feature_model(const std::string& name, const std::string &path_to_parameters):
    GMM_model(name),gmm(path_to_parameters,"bla")
{
   x.resize(6);
}

float GMM_feature_model::predict(arma::colvec3& force,arma::colvec3& torque){
    x(arma::span(0,2)) = force;
    x(arma::span(3,5)) = torque;
    return 0;//gmm.likelihood(x);
}


}
