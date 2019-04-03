#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <birds_eye/AvoidObstacleConfig.h>

#include <vector>



/**
 * Lane Follow
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class AvoidObstacle
{
public:
    AvoidObstacle();
    void scanCb(const sensor_msgs::LaserScan& msg);
    void configCallback(birds_eye::AvoidObstacleConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber scan_sub_;

    dynamic_reconfigure::Server<birds_eye::AvoidObstacleConfig> server_;
    birds_eye::AvoidObstacleConfig config_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the AvoidObstacle object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
AvoidObstacle::AvoidObstacle() : nh_{"~"}
{
    std::string scan_sub_topic;
    if (!nh_.getParam("scan_topic", scan_sub_topic))
    {
        ROS_ERROR_STREAM("Please set param 'scan_topic'");
        scan_sub_topic = "/scan";
    }
    else
    {
        ROS_INFO_STREAM("Using scan topic '" << scan_sub_topic << "'");
    }

    // Subscribe to the camera publisher node
    scan_sub_ = nh_.subscribe(scan_sub_topic, 5, &AvoidObstacle::scanCb, this);

    // Publish on the twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&AvoidObstacle::configCallback, this, _1, _2));

    // Load defaults
    server_.getConfigDefault(config_);
}



/**
 * Dynamic Reconfigure Callback
 * ============================
 *
 * This function is called every time the Dynamic Reconfigure UI
 *   is updated by the user.
 */
void AvoidObstacle::configCallback(birds_eye::AvoidObstacleConfig &config, uint32_t level)
{
    config_ = config;
}




/**
 * Callback function
 * =================
 *
 * Called once every time a image is published on the topic this
 *   node is subscribed to. The image is passed to the function as
 *   a ImageConstPtr
 */
void AvoidObstacle::scanCb(const sensor_msgs::LaserScan& msg)
{
    const float cap = config_.dist_cap;
    std::vector<float> points = msg.ranges;

    // Calculate weighted values
    const int len = points.size();
    float sum = 0.0f;
    for (int i = 0; i < points.size(); i++)
    {
        // Cap and flip
        float val = (cap - std::min(cap, points[i])) / cap;

        // weight
        float w = std::min(i, len - i); // distance from edge
        w *= 2.0 / len;               // normalize 0..1
        if (i > len/2) w *= -1;     // right side is negative

        // calculate sum
        sum += val  * (1.0/len) * w;
    }


    // If the number of white pixels is above a certain percent, stop
    geometry_msgs::Twist twist;
    twist.linear.x = config_.drive_speed;
    twist.angular.z = std::min(1.0, config_.turn_multip * sum);

    pub_.publish(twist);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_obstacle");

    // Create a AvoidObstacle object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    AvoidObstacle sd{};

    ROS_INFO_STREAM("avoid_obstacle running!");
    ros::spin();
    return 0;
}
