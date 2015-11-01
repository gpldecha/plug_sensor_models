#ifndef BASE_PEG_CLASSIFIER_H_
#define BASE_PEG_CLASSIFIER_H_

#include <armadillo>


/**
 *  Base class for peg classifier
 *
 *  All classifiers which take force and torque signal from the FT sensor and return
 *  a classification result (contact/no contact, etc..) have to inherit and implement
 *  the update funcion.
 *
 */

namespace psm{

class Base_peg_classifier{

public:

    virtual void update(arma::colvec& Y,const arma::fcolvec3& force, const arma::fcolvec3& torque) = 0;

};

}

#endif
