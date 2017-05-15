//
//  KalmanFilter.hpp
//  kalmann-filter
//
//  Created by Pau Cardona on 13/05/2017.
//
//

#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp

#include <stdio.h>

class KalmanFilter
{
public:
    KalmanFilter(float initialPosition, float initialVelocity, float sensorVariance_);
    ~KalmanFilter();
    
    void step(float positionMeasurement, float timeInterval);
    float normalizeAngle(float angle);
    float angleDifference(float leftAngle, float rightAngle);
    float getPositionEstimate();
    float getVelocityEstimate();
private:
    
    float positionKalmanGain;
    float velocityKalmanGain;
        
    float positionEstimate;
    float velocityEstimate;
};

#endif /* KalmanFilter_hpp */
