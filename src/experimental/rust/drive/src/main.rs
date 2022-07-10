use rosrust;
const TRACK_RADIUS : f64 = 10.0; // meter
const WHEEL_RADIUS : f64 = 0.5; // meter

rosrust::rosmsg_include!(mrover/Chassis, geometry_msgs/Twist);

fn main() {
    rosrust::init("drive");
    let wheel_cmd_publisher = rosrust::publish("/cmd_wheel", 100).unwrap();

    // Subscriber lifetime is dicatated by RAII
    let sub = rosrust::subscribe(
       "/cmd_vel", 100,
       move | vel_msg : geometry_msgs::Twist | {
            // Get linear and angular velocities
            let v = vel_msg.linear.x;
            let omega = vel_msg.angular.z;

            // Transform into L & R wheel angular velocities using diff drive kinematics
            let omega_l = (v - omega * TRACK_RADIUS / 2.0)/WHEEL_RADIUS;
            let omega_r = (v + omega * TRACK_RADIUS / 2.0)/WHEEL_RADIUS;

            // Publish to /cmd_wheel
            let mut wheel_cmd = mrover::Chassis::default();
            wheel_cmd.omega_l = omega_l;
            wheel_cmd.omega_r = omega_r;
            wheel_cmd_publisher.send(wheel_cmd).unwrap();

       } 
    ).unwrap();

    rosrust::spin();
}
