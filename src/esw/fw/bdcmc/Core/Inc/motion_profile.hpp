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
                      AccelerationUnit maxAcceleration) : mMaxVelocity{maxVelocity},
                                                         mMaxAcceleration{maxAcceleration},
                                                         mTotalDistance{initialPosition - desiredPosition},
                                                         mTAccelerationDone{mMaxVelocity / mMaxAcceleration},
                                                         mTEnd{mTAccelerationDone + mTotalDistance / mMaxVelocity},
                                                         mTCruiseDone{mTEnd - mTAccelerationDone}
        {}

        void reset () {
            mT = 0;
        }

        void update(TimeUnit dt) {
            mT += dt;
        }

        auto velocity() -> VelocityUnit {
            if (mT >= 0 && mT < mTAccelerationDone) {
                return mMaxAcceleration * mT;
            } else if (mT >= mTAccelerationDone && mT < mTCruiseDone) {
                return mMaxVelocity;
            } else if (mT >= mTCruiseDone && mT <= mTEnd) {
                return -mMaxAcceleration * mT + mMaxAcceleration * mTEnd;
            } else {
                return 0.0;
            }
        }

        auto is_finished() -> bool {
            return mT >= mTEnd;
        }

        auto t() -> TimeUnit {
            return mT;
        }

    private:
        const VelocityUnit mMaxVelocity{};
        const AccelerationUnit mMaxAcceleration{};

        TimeUnit mT{0};

        const PositionUnit mTotalDistance{};
        const TimeUnit mTAccelerationDone{};
        const TimeUnit mTEnd{};
        const TimeUnit mTCruiseDone{};

    };
}