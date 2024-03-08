class TrapezoidalMotionProfile {
public:
    // velocity of the profile at time t
    double velocity(double t);

private:
    double mMaxVelocity;
    double mMaxAcceleration;
    
    double mInitialPosition;
    double mDesiredPosition;
};