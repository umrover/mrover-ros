class MotionProfile {
public:
    MotionProfile(double initialPosition,
                  double desiredPosition,
                  double maxVelocity,
                  double maxAcceleration) : mInitialPosition{initialPosition},
                                            mDesiredPosition{desiredPosition},
                                            mMaxVelocity{maxVelocity},
                                            mMaxAcceleration{maxAcceleration} {}

    // velocity of the profile at time t
    [[nodiscard]] virtual double velocity(double t) const = 0;

    void update(double initialPosition,
                double desiredPosition) {
        mInitialPosition = initialPosition;
        mDesiredPosition = desiredPosition;
    }


protected:
    double mInitialPosition;
    double mDesiredPosition;

    double mMaxVelocity;
    double mMaxAcceleration;
};

class TrapezoidalMotionProfile : public MotionProfile {
public:
    TrapezoidalMotionProfile(double initialPosition,
                                          double desiredPosition,
                                          double maxVelocity,
                                          double maxAcceleration) : MotionProfile(initialPosition,
                                                                     desiredPosition,
                                                                     maxVelocity,
                                                                     maxAcceleration) {}
    [[nodiscard]] double velocity(double t) const override;
};