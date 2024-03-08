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

        void update(PositionUnit initialPosition,
                    PositionUnit desiredPosition) {
            mInitialPosition = initialPosition;
            mDesiredPosition = desiredPosition;
        }

        // TODO: build in way to keep track of start time to class
        [[nodiscard]] double velocity(TimeUnit t) const override {
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

    protected:
        PositionUnit mInitialPosition;
        PositionUnit mDesiredPosition;

        VelocityUnit mMaxVelocity;
        AccelerationUnit mMaxAcceleration;
    };
}