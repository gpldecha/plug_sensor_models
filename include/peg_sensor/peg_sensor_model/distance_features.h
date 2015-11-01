#ifndef PEG_SENSOR_DISTANCE_FEATURES_H_
#define PEG_SENSOR_DISTANCE_FEATURES_H_

#include "wrapobject.h"

namespace psm{

class Distance_features{

public:

    Distance_features(wobj::WrapObject &wrapped_world);

    void compute_surface_edge_vector(const arma::fcolvec3 &P);

public:

     arma::fcolvec3 point_surface;
     arma::fcolvec3 point_edge;
     bool           bIsInside;

private:

    wobj::WrapObject        wrapped_world;

};

}

#endif
