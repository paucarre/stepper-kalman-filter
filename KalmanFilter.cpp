#include "KalmanFilter.hpp"


KalmanFilter::KalmanFilter(float initialPosition, float initialVelocity, float positionKalmanGain_):
positionEstimate(initialPosition),
velocityEstimate(initialVelocity),
positionKalmanGain(positionKalmanGain_)
{
}

KalmanFilter::~KalmanFilter(){
    
}

float KalmanFilter::getPositionEstimate(){
    return positionEstimate;
}

float KalmanFilter::getVelocityEstimate(){
    return velocityEstimate;
}

void KalmanFilter::step(float positionMeasurement, float timeInterval)
{
 
    // State update using system simulation
    // Xt|t-1 = F * Xt-1|t-1 + B * Ut (note: B is zero and velocity estimate is naive, that might need to change)
    //TODO: just simply use the previous position plus the steps moved by the stepper
    float systemPositionEstimate = normalizeAngle(positionEstimate + (timeInterval * velocityEstimate));
    
    // State update using kalman gain and sensor data
    // Xt|t = Xt|t-1 + Kt * (sensor_position - H * Xt|t-1)
    float previousPositionEstimate = positionEstimate;
    positionEstimate = normalizeAngle(systemPositionEstimate + ( positionKalmanGain * angleDifference( positionMeasurement, systemPositionEstimate ) ));
    velocityEstimate = angleDifference(positionEstimate, previousPositionEstimate) / timeInterval;
    
}

float KalmanFilter::normalizeAngle(float angle)
{
    //TODO: make this properly using modulus
    while(angle < 0.0 || angle > 360.0) {
        if(angle < 0.0){
            angle += 360.0;
        } else if(angle > 360.0) {
            angle -= 360.0;
        }
    }
    return angle;
}

/*
 * Angular difference fullfills the following requirements:
 *    5 - 350 =  15
 *  350 -   5 = -15
 *   60 -  80 = -20
 *   80 -  60 =  20
 */
float KalmanFilter::angleDifference(float leftAngle, float rightAngle)
{
    float difference = leftAngle - rightAngle;
    if(difference > 180.0){
        difference -= 360;
    } else if(difference < - 180.0) {
        difference += 360;
    }
    return difference;
}
