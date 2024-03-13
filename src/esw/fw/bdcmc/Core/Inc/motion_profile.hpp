#include "units/units.hpp"

namespace mrover {
    template<IsUnit PositionUnit = Radians, IsUnit TimeUnit = Seconds>
    class TrapezoidalMotionProfile {
        using VelocityUnit = compound_unit<PositionUnit, inverse<TimeUnit>>;
        using AccelerationUnit = compound_unit<VelocityUnit, inverse<TimeUnit>>;
    public:
        TrapezoidalMotionProfile(PositionUnit initialPosition,
                      PositionUnit desiredPosition,
                      VelocityUnit maxVelocity,
                      AccelerationUnit maxAcceleration) : mInitialPosition{initialPosition},
                                                          mDesiredPosition{desiredPosition},
                                                          mMaxVelocity{maxVelocity},
                                                          mMaxAcceleration{maxAcceleration} {}

        void reset () {
            mT = 0;
        }

        void reset(PositionUnit initialPosition,
                    PositionUnit desiredPosition) {
            mInitialPosition = initialPosition;
            mDesiredPosition = desiredPosition;
            reset();
        }

        void update(TimeUnit dt) {
            mT += dt;
        }

        auto velocity() -> VelocityUnit {
            double totalDistance = (mDesiredPosition - mInitialPosition);
            double timeToAccelerate = mMaxVelocity / mMaxAcceleration;

            double tAccelerationDone = timeToAccelerate;
            double tEnd = tAccelerationDone + totalDistance / mMaxVelocity;
            double tCruiseDone = tEnd - tAccelerationDone;

            if (mT >= 0 && mT < tAccelerationDone) {
                return mMaxAcceleration * mT;
            } else if (mT >= tAccelerationDone && mT < tCruiseDone) {
                return mMaxVelocity;
            } else if (mT >= tCruiseDone && mT <= tEnd) {
                return -mMaxAcceleration * mT + mMaxAcceleration * tEnd;
            } else {
                return 0.0;
            }
        }

    private:
        PositionUnit mInitialPosition;
        PositionUnit mDesiredPosition;

        VelocityUnit mMaxVelocity;
        AccelerationUnit mMaxAcceleration;

        TimeUnit mT = 0;
    };
}