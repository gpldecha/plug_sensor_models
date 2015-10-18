#ifndef FEATURE_MODEL_H_
#define FEATURE_MODEL_H_

#include <string>
#include <armadillo>
#include <statistics/distributions/gmm.h>

namespace psm {

class GMM_model{
public:
    GMM_model(const std::string& name);
    virtual float predict(arma::colvec3& force,arma::colvec3& torque) const = 0;

protected:
    std::string name;
};


class GMM_feature_model : public GMM_model{

public:

    GMM_feature_model(const std::string& name,const std::string& path_to_parameters);

    float predict(arma::colvec3& force,arma::colvec3& torque);

private:

    GMM       gmm;
    arma::vec x;

};


}
#endif
