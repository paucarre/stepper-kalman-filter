//
//  KalmanGain.hpp
//  KalmanFilter
//
//  Created by Pau Cardona on 14/05/2017.
//  Copyright © 2017 Pau Carre. All rights reserved.
//

#ifndef KalmanGain_hpp
#define KalmanGain_hpp

#include <stdio.h>

class KalmanGain
{
public:
    ~KalmanGain();
    static KalmanGain compute(float sensorVariance,
                              float timeInterval,
                              float convergenceToleranceError,
                              float positionCovariance,
                              float velocityCovariance,
                              float positionVelocityCovariance,
                              float systemPositionCovariance);
    float getPositionKalmanGain();
    float getVelocityKalmanGain();
    void print();
    
private:
    KalmanGain(float sensorVariance,
               float timeInterval,
               float convergenceToleranceError,
               float positionCovariance,
               float velocityCovariance,
               float positionVelocityCovariance,
               float positionKalmanGain,
               float velocityKalmanGain,
               float systemPositionCovariance);
    float kalmanQuadraticDifference(float previousPositionGain, float previousVelocityGain);
    
    void step();
    
    float timeInterval;
    
    float systemPositionCovariance; // Q matrix
    
    float convergenceToleranceError;
    
    float sensorVariance;
    
    // Covariance matrix
    float positionCovariance;
    float velocityCovariance;
    float positionVelocityCovariance;
    
    // Kalman gains
    float positionKalmanGain;
    float velocityKalmanGain;
};


#endif /* KalmanGain_hpp */
