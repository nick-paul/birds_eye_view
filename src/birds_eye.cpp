#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <birds_eye/BirdsEyeConfig.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <utility> // pair
#include <cmath>

class BirdsEye
{
public:
    BirdsEye();
    void scanCb(const sensor_msgs::LaserScan& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(birds_eye::BirdsEyeConfig &config, uint32_t level);
    void birdsEye(const cv::Mat& source, cv::Mat& destination);

private:
    void colorMask(const cv::Mat &in);
    void makeOccupancyGrid(const cv::Mat &in, const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& grid);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // Subscribers
    image_transport::Subscriber image_sub_;
    ros::Subscriber scan_sub_;

    // Publishers
    image_transport::Publisher birds_eye_pub_;
    image_transport::Publisher color_mask_pub_;
    ros::Publisher grid_pub_;

    dynamic_reconfigure::Server<birds_eye::BirdsEyeConfig> server_;
    birds_eye::BirdsEyeConfig config_;

    // Last scan message
    sensor_msgs::LaserScan scan_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the BirdsEye object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
BirdsEye::BirdsEye() :
    nh_{"~"},
    it_{nh_}
{
    std::string cam_sub_topic;
    if (!nh_.getParam("camera_topic", cam_sub_topic))
    {
        ROS_ERROR_STREAM("Please set param 'camera_topic'");
        cam_sub_topic = "/camera";
    }

    std::string scan_sub_topic;
    if (!nh_.getParam("scan_topic", scan_sub_topic))
    {
        ROS_ERROR_STREAM("Please set param 'scan_topic'");
        scan_sub_topic = "/scan";
    }

    // Subscribe to the camera publisher node
    image_sub_ = it_.subscribe(cam_sub_topic, 1, &BirdsEye::imageCb, this);
    scan_sub_  = nh_.subscribe(scan_sub_topic, 10, &BirdsEye::scanCb, this);


    birds_eye_pub_ = it_.advertise("birds_eye", 1);
    color_mask_pub_ = it_.advertise("color_mask", 1);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&BirdsEye::configCallback, this, _1, _2));

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
void BirdsEye::configCallback(birds_eye::BirdsEyeConfig &config, uint32_t level)
{
    config_ = config;
}


void BirdsEye::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg_out;

    // Compute and publish birds eye
    birdsEye(cv_ptr->image, frame);
    msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    birds_eye_pub_.publish(msg_out);

    // Compute and publish color mask
    colorMask(frame);
    msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    color_mask_pub_.publish(msg_out);

    // Create occupancy grid
    nav_msgs::OccupancyGrid grid;
    makeOccupancyGrid(frame, scan_, grid);
    grid_pub_.publish(grid);

}


void BirdsEye::makeOccupancyGrid(const cv::Mat &in, const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& grid)
{
    // Convert image to greyscale
    cv::Mat gray;
    cv::cvtColor(in, gray, cv::COLOR_BGR2GRAY);

    cv::Size image_size = gray.size();
    const int image_width  = image_size.width;
    const int image_height = image_size.height;

    const int width = image_height / 4;
    const int height = image_width / 4;


    double width_ratio  = (double)image_height / (double)width;
    double height_ratio = (double)image_width / (double)height;

    assert(width  < image_width);
    assert(height < image_height);


    // Build Grid 
    const double resolution = config_.grid_res;
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;

    // Origin
    grid.info.origin.position.x = config_.grid_origin_x;
    grid.info.origin.position.y = config_.grid_origin_y;

    grid.data.resize(width * height);

    // Add pixels to grid
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            const int row = x * width_ratio;
            const int col = y * height_ratio;
            const uint8_t pixel = gray.at<uchar>(row, col);

            grid.data[x + y*width] = pixel == 0 ? 0 : 127;
        }
    }


    // LaserScan
    auto ranges = scan.ranges;
    int offset = config_.scan_offset_deg;
    std::vector<std::pair<int, int>> xy_points;

    // Convert to xy coords, assumes 1 deg <-> 1 point resolution
    for (int deg = 0; deg < ranges.size(); deg++)
    {
        double theta = (double)(deg + offset) * (double)M_PI / 180.0;
        double radius  = ranges[deg];

        double x = radius * std::cos(theta) - grid.info.origin.position.x;
        double y = radius * std::sin(theta) - grid.info.origin.position.y;

        // Add to occupancy grid
        int i = std::floor((1/resolution) * x);
        int j = std::floor((1/resolution) * y);

        // Dilate Pixels
        i -= 1;
        j -= 1;
        for (int di = 0; di < 3; di++)
        {
            for (int dj = 0; dj < 3; dj++)
            {
                if ((i+di) < width && (j+dj) < height && (i+di) > 0 && (j+dj) > 0)
                {
                    grid.data[(i+di) + (j+dj)*width] = 100; // different number than image, still non-zero
                }
            }
        }
    }
}



/*
 * Project an image into "Bird's Eye" perspective
 */
void BirdsEye::birdsEye(const cv::Mat& source, cv::Mat& destination)
{
    using namespace cv;

    double alpha        = (config_.alpha - 90) * M_PI/180;
    double beta         = 0; // (config_.beta -90) * M_PI/180;
    double gamma        = 0; // (config_.gamma -90) * M_PI/180;
    double dist         = config_.dist;
    double focalLength  = config_.f;

    Size image_size = source.size();
    double w = (double)image_size.width;
    double h = (double)image_size.height;

    // Projecion matrix 2D -> 3D
    Mat A1 = (Mat_<float>(4, 3)<< 
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0, 0,
        0, 0, 1 );


    // Rotation matrices Rx, Ry, Rz

    Mat RX = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha), cos(alpha), 0,
        0, 0, 0, 1 );

    Mat RY = (Mat_<float>(4, 4) << 
        cos(beta), 0, -sin(beta), 0,
        0, 1, 0, 0,
        sin(beta), 0, cos(beta), 0,
        0, 0, 0, 1  );

    Mat RZ = (Mat_<float>(4, 4) << 
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma), cos(gamma), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1  );


    // R - rotation matrix
    Mat R = RX * RY * RZ;

    // T - translation matrix
    Mat T = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,  
        0, 1, 0, 0,  
        0, 0, 1, dist,  
        0, 0, 0, 1); 
    
    // K - intrinsic matrix 
    Mat K = (Mat_<float>(3, 4) << 
        focalLength, 0, w/2, 0,
        0, focalLength, h/2, 0,
        0, 0, 1, 0
        ); 


    Mat transformationMat = K * (T * (R * A1));

    warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
}

/*
 * Simply save the scan over
 */
void BirdsEye::scanCb(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
}

/*
 * Apply the color mask
 */
void BirdsEye::colorMask(const cv::Mat &mat)
{
    // Blur the image to reduce noise (kernel must be odd)
    if (config_.use_median_blur)
    {
        cv::medianBlur(mat, mat, 2*config_.median_blur_amount + 1);
    }

    // Convert input image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(mat, hsv_image, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels(3);
    cv::split(mat, channels);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image,
            cv::Scalar(config_.mask_l_hue, config_.mask_l_sat, config_.mask_l_lum),
            cv::Scalar(config_.mask_h_hue, config_.mask_h_sat, config_.mask_h_lum),
            upper_red_hue_range);

    cv::Mat dilated;
    cv::Mat dilate_element =\
                cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                cv::Size(2*config_.mask_dialate + 1, 2*config_.mask_dialate+1),
                                cv::Point(config_.mask_dialate, config_.mask_dialate));

    cv::dilate(upper_red_hue_range, dilated, dilate_element);

    channels[0] &= dilated;
    channels[1] &= dilated;
    channels[2] &= dilated;
    cv::merge(channels, mat);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "birds_eye");

    // Create a BirdsEye object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    BirdsEye sd{};

    ROS_INFO_STREAM("birds_eye running!");
    ros::spin();
    return 0;
}
