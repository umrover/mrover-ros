class TrapezoidalMotionProfile {
public:
    // velocity of the profile at time t
    double velocity(double t);

private:
    double mMaxVelocity;
    double mMaxAcceleration;
    
    double mInitialPosition;
    double mDesiredPosition;

    // TODO: figure out a good way to expose either cruise distance or time
    // maybe an overloaded constructor based on units, although we should also templatize 
    // any valid base position unit (m -> m/s -> m/s^2, in -> in/s -> in/s^2)

    // distance/time that constant (maximum) velocity is maintained
    double mCruiseDistance;
    double mCruiseTime;
};