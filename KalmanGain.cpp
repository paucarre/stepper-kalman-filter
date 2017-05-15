//
//  KalmanGain.cpp
//  KalmanFilter
//
//  Created by Pau Cardona on 14/05/2017.
//  Copyright © 2017 Pau Carre. All rights reserved.
//

#include "KalmanGain.hpp"


KalmanGain::~KalmanGain(){
    
}

KalmanGain KalmanGain::compute(float sensorVariance,
                               float timeInterval,
                               float convergenceToleranceError,
                               float positionCovariance,
                               float velocityCovariance,
                               float positionVelocityCovariance,
                               float systemPositionCovariance
)
{
    // Covariance matrix of poisition and velocity
    KalmanGain kalmanGain( sensorVariance,
                           timeInterval,
                           convergenceToleranceError,
                           positionCovariance,
                           velocityCovariance,
                           positionVelocityCovariance,
                           0.0,
                           0.0,
                           systemPositionCovariance);
    bool kalmanGainConverged = false;
    float previousPositionGain = kalmanGain.positionKalmanGain;
    float previousVelocityGain = kalmanGain.velocityKalmanGain;
    int currentIteration = 0;
    int maximumIterations = 1000;
    while(!kalmanGainConverged && currentIteration < maximumIterations){
        kalmanGain.step();
        kalmanGainConverged = kalmanGain.kalmanQuadraticDifference(previousPositionGain, previousVelocityGain) < convergenceToleranceError;
        currentIteration++;
        printf("(%f, %f) -> (%f, %f)\n", previousPositionGain, previousVelocityGain, kalmanGain.positionKalmanGain, kalmanGain.velocityKalmanGain);
        previousPositionGain = kalmanGain.positionKalmanGain;
        previousVelocityGain = kalmanGain.velocityKalmanGain;
    }
    return kalmanGain;
}

void KalmanGain::print()
{
    printf("(%f, %f)\n", positionKalmanGain, velocityKalmanGain);
}

float KalmanGain::getPositionKalmanGain()
{
    return positionKalmanGain;
}

float KalmanGain::getVelocityKalmanGain()
{
    return velocityKalmanGain;
}


float KalmanGain::kalmanQuadraticDifference(float previousPositionGain, float previousVelocityGain)
{
    float positionKalmanGainDifference = (positionKalmanGain - previousPositionGain);
    float velocityKalmanGainDifference = (velocityKalmanGain - previousVelocityGain);
    return (positionKalmanGainDifference * positionKalmanGainDifference) + (velocityKalmanGainDifference * velocityKalmanGainDifference);
}

void KalmanGain::step()
{
    /*
     * F = system matrix
     * P = moving state covairance
     * Q = state variance (set to zero for now)
     * H = position extractor. Matrix (1 0)
     * K = Kalman gain (one single number as there is only one position sensor)
     * B = effect of external action in system's state
     * X = system state. Vector (position velocity)
     */
    
    // Covariance update using system simulation
    // Pt|t-1 = F * Pt-1|t-1 * F' + Q
    positionCovariance = positionCovariance + ( timeInterval * ( ( 2.0 * positionVelocityCovariance ) + ( timeInterval * velocityCovariance ) ) ) + systemPositionCovariance;
    // Note that: velocityCovariance = velocityCovariance;
    positionVelocityCovariance = positionVelocityCovariance + (timeInterval * velocityCovariance);
    
    // Kalman gain update
    // Kt = Pt|t-t * H' * inv(H * Pt|t-1 * H' + R)
    velocityKalmanGain = positionVelocityCovariance / (positionCovariance + sensorVariance);
    positionKalmanGain = positionCovariance / (positionCovariance + sensorVariance);
    
    // Coviariance update using sensor data
    // Pt|t = Pt|t-1 - Kt * H * Pt|t-1
    positionCovariance = positionCovariance - (positionKalmanGain * positionCovariance);
    velocityCovariance = velocityCovariance - (velocityKalmanGain * positionCovariance);
    positionVelocityCovariance = positionVelocityCovariance - (velocityKalmanGain * positionVelocityCovariance);
}


KalmanGain::KalmanGain(float sensorVariance_,
                       float timeInterval_,
                       float convergenceToleranceError_,
                       float positionCovariance_,
                       float velocityCovariance_,
                       float positionVelocityCovariance_,
                       float positionKalmanGain_,
                       float velocityKalmanGain_,
                       float systemPositionCovariance_):sensorVariance(sensorVariance_),timeInterval(timeInterval_),convergenceToleranceError(convergenceToleranceError_),positionCovariance(positionCovariance_),velocityCovariance(velocityCovariance_),positionVelocityCovariance(positionVelocityCovariance_),positionKalmanGain(positionKalmanGain_),
                           systemPositionCovariance(systemPositionCovariance_){
}
