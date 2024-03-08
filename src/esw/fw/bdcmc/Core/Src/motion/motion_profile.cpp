#include "motion_profile.hpp"

double TrapezoidalMotionProfile::velocity(double t) {
    double totalDistance = (mDesiredPosition - mInitialPosition);
    double timeToAccelerate = mMaxVelocity / mMaxAcceleration;

    double tAccelerationDone = timeToAccelerate;
    double tEnd = tAccelerationDone + totalDistance / mMaxVelocity;
    double tCruiseDone = tEnd - tAccelerationDone;

    if (t >= 0 && t < tAccelerationDone) {
        return mMaxAcceleration * t;
    } else if (t >= tAccelerationDone && t < tCruiseDone) {
        return mMaxVelocity;
    } else if (t >= tCruiseDone && t <= tEnd) {
        return -mMaxAcceleration * t + mMaxAcceleration * tEnd;
    } else {
        return 0.0;
    }
}