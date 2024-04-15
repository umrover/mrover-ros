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
            if (mT >= TimeUnit{0} && mT < mTAccelerationDone) {
                return mMaxAcceleration * mT;
            } else if (mT >= mTAccelerationDone && mT < mTCruiseDone) {
                return mMaxVelocity;
            } else if (mT >= mTCruiseDone && mT <= mTEnd) {
                return -mMaxAcceleration * mT + mMaxAcceleration * mTEnd;
            } else {
                return VelocityUnit{0};
            }
        }

        auto is_finished() -> bool {
            return mT >= mTEnd;
        }

        auto t() -> TimeUnit {
            return mT;
        }

    private:
        VelocityUnit mMaxVelocity{};
        AccelerationUnit mMaxAcceleration{};

        TimeUnit mT{0};

        PositionUnit mTotalDistance{};
        TimeUnit mTAccelerationDone{};
        TimeUnit mTEnd{};
        TimeUnit mTCruiseDone{};
    };
}