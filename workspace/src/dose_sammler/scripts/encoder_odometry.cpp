#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pigpio.h>
#include <cmath>
#include <map>
#include <string>

// Encoder pins
const std::map<std::string, std::pair<int, int>> ENCODER_PINS = {
    {"FL", {19, 13}},
    {"FR", {11, 0}},
    {"RL", {22, 27}},
    {"RR", {24, 23}}
};

// Tick counter
std::map<std::string, int> ticks = {{"FL", 0}, {"FR", 0}, {"RL", 0}, {"RR", 0}};
std::map<std::string, int> last_states;

// Callback function for encoder interrupts
void encoderCallback(int gpio, int level, uint32_t tick, const std::string& wheel_name, int pin_a, int pin_b) {
    int a = gpioRead(pin_a);
    int b = gpioRead(pin_b);
    int current_state = (a << 1) | b;

    int last_state = last_states[wheel_name];
    int transition = (last_state << 2) | current_state;

    // Valid transitions for forward and backward movement
    if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000) {
        ticks[wheel_name]++;
    } else if (transition == 0b0010 || transition == 0b0100 || transition == 0b1101 || transition == 0b1011) {
        ticks[wheel_name]--;
    }

    last_states[wheel_name] = current_state;
}

// Setup encoders
void setupEncoders() {
    if (gpioInitialise() < 0) {
        ROS_ERROR("Failed to initialize pigpio library");
        ros::shutdown();
        return;
    }

    for (const auto& [wheel_name, pins] : ENCODER_PINS) {
        int pin_a = pins.first;
        int pin_b = pins.second;

        gpioSetMode(pin_a, PI_INPUT);
        gpioSetMode(pin_b, PI_INPUT);
        gpioSetPullUpDown(pin_a, PI_PUD_UP);
        gpioSetPullUpDown(pin_b, PI_PUD_UP);

        last_states[wheel_name] = (gpioRead(pin_a) << 1) | gpioRead(pin_b);

        gpioSetAlertFuncEx(pin_a, [](int gpio, int level, uint32_t tick, void* userdata) {
            auto* data = static_cast<std::pair<std::string, std::pair<int, int>>*>(userdata);
            encoderCallback(gpio, level, tick, data->first, data->second.first, data->second.second);
        }, new std::pair<std::string, std::pair<int, int>>(wheel_name, pins));

        gpioSetAlertFuncEx(pin_b, [](int gpio, int level, uint32_t tick, void* userdata) {
            auto* data = static_cast<std::pair<std::string, std::pair<int, int>>*>(userdata);
            encoderCallback(gpio, level, tick, data->first, data->second.first, data->second.second);
        }, new std::pair<std::string, std::pair<int, int>>(wheel_name, pins));
    }
}

// Calculate odometry
std::tuple<double, double, double, std::map<std::string, double>> calculateOdometry(const std::map<std::string, int>& ticks_delta, double dt) {
    const double TICKS_PER_REV = 2797;
    const double WHEEL_RADIUS = 0.044;
    const double L = 0.244;
    const double W = 0.132;

    std::map<std::string, double> wheel_speeds;
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS;

    for (const auto& [wheel_name, delta] : ticks_delta) {
        double revs = delta / TICKS_PER_REV;
        wheel_speeds[wheel_name] = (wheel_circumference * revs) / dt;
    }

    double vx = (wheel_speeds["FL"] + wheel_speeds["FR"] + wheel_speeds["RL"] + wheel_speeds["RR"]) / 4;
    double vy = (-wheel_speeds["FL"] + wheel_speeds["FR"] + wheel_speeds["RL"] - wheel_speeds["RR"]) / 4;
    double wz = (-wheel_speeds["FL"] - wheel_speeds["FR"] + wheel_speeds["RL"] + wheel_speeds["RR"]) / (4 * (L + W));

    return {vx, vy, wz, wheel_speeds};
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "encoder_odometry");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/wheel_states", 10);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    setupEncoders();

    double x = 0.0, y = 0.0, th = 0.0;
    std::map<std::string, int> last_ticks = ticks;
    ros::Time last_time = ros::Time::now();
    ros::Rate rate(100); // 100 Hz

    try {
        while (ros::ok()) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_time).toSec();
            last_time = now;

            if (dt == 0) {
                rate.sleep();
                continue;
            }

            std::map<std::string, int> ticks_delta;
            for (const auto& [wheel_name, tick] : ticks) {
                ticks_delta[wheel_name] = tick - last_ticks[wheel_name];
            }
            last_ticks = ticks;

            auto [vx, vy, vth, wheel_speeds] = calculateOdometry(ticks_delta, dt);

            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            geometry_msgs::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = -x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0;

            // Fix quaternion calculation
            tf2::Quaternion q;
            q.setRPY(0, 0, th);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster.sendTransform(t);

            nav_msgs::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = t.transform.rotation;
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;
            odom_pub.publish(odom);

            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = now;
            joint_state.name = {"FL", "FR", "RL", "RR"};
            joint_state.position = {0.0, 0.0, 0.0, 0.0};
            joint_state.velocity = {wheel_speeds["FL"], wheel_speeds["FR"], wheel_speeds["RL"], wheel_speeds["RR"]};
            joint_state.effort = {0.0, 0.0, 0.0, 0.0};
            joint_pub.publish(joint_state);

            rate.sleep();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    gpioTerminate();
    return 0;
}