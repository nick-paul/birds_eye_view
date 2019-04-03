#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <birds_eye/LaneFollowConfig.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <utility> // pair
#include <cmath>

//static constexpr double M_PI = 3.1415926595;


// Change this value if you are subscribing to a different camera
#define CVWIN_PREVIEW "thresh preview"

/**
 * Lane Follow
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class LaneFollow
{
public:
    LaneFollow();
    ~LaneFollow();
    void scanCb(const sensor_msgs::LaserScan& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(birds_eye::LaneFollowConfig &config, uint32_t level);
    void birdsEye(const cv::Mat& source, cv::Mat& destination);

private:
    std::pair<float, float> getVel(const cv::Mat& src, cv::Mat& dst);
    void colorMask(const cv::Mat &in);
    void makeOccupancyGrid(const cv::Mat &in, const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& grid);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;
    ros::Subscriber scan_sub_;

    image_transport::Publisher birds_eye_pub_;
    image_transport::Publisher color_mask_pub_;
    ros::Publisher grid_pub_;

    dynamic_reconfigure::Server<birds_eye::LaneFollowConfig> server_;
    birds_eye::LaneFollowConfig config_;

    sensor_msgs::LaserScan scan_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the LaneFollow object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
LaneFollow::LaneFollow() :
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
    image_sub_ = it_.subscribe(cam_sub_topic, 1, &LaneFollow::imageCb, this);
    scan_sub_  = nh_.subscribe(scan_sub_topic, 10, &LaneFollow::scanCb, this);

    // Publish on the twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);

    birds_eye_pub_ = it_.advertise("birds_eye", 1);
    color_mask_pub_ = it_.advertise("color_mask", 1);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&LaneFollow::configCallback, this, _1, _2));

    // Load defaults
    server_.getConfigDefault(config_);
}



/**
 * Destructor
 * ==========
 *
 * Destroy CV windows
 */
LaneFollow::~LaneFollow()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}

/**
 * Dynamic Reconfigure Callback
 * ============================
 *
 * This function is called every time the Dynamic Reconfigure UI
 *   is updated by the user.
 */
void LaneFollow::configCallback(birds_eye::LaneFollowConfig &config, uint32_t level)
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
void LaneFollow::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    nav_msgs::OccupancyGrid grid;
    makeOccupancyGrid(frame, scan_, grid);
    grid_pub_.publish(grid);

}


void LaneFollow::makeOccupancyGrid(const cv::Mat &in, const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& grid)
{

    // Convert image to greyscale
    cv::Mat gray;
    cv::cvtColor(in, gray, cv::COLOR_BGR2GRAY);

    const int width = 128;
    const int height = 128;

    cv::Size image_size = in.size();
    const int image_width  = image_size.width;
    const int image_height = image_size.height;

    double width_ratio  = (double)image_width / (double)width;
    double height_ratio = (double)image_height / (double)height;

    assert(width  < image_width);
    assert(height < image_height);


    // Build Grid 

    const double resolution = 0.01;
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;

    // Origin
    grid.info.origin.position.x = config_.grid_origin_x;
    grid.info.origin.position.y = config_.grid_origin_y;

    grid.data.resize(width * height);

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            const uint8_t pixel = gray.at<uchar>(
                    std::floor(i * height_ratio), 
                    std::floor(j * width_ratio));
            grid.data[i + j*width] = pixel == 0 ? 0 : 255;
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

        x += config_.scan_offset_x;
        y += config_.scan_offset_y;
        double scale_shift = config_.scan_scale_shift;

        // Add to occupancy grid
        int i = std::floor((1/resolution) * x * scale_shift);
        int j = std::floor((1/resolution) * y * scale_shift);

        //ROS_ERROR_STREAM(" deg=" << deg << " theta=" << theta << " rad=" << radius << " xy=" << x <<"," << y << " ij=" <<i << ", " << j); 
        i -= 1;
        j -= 1;
        for (int di = 0; di < 3; di++)
        {
            for (int dj = 0; dj < 3; dj++)
            {
                if ((i+di) < width && (j+dj) < width && (i+di) > 0 && (j+dj) > 0)
                {
                    grid.data[(i+di) + (j+dj)*width] = 100; // different number than image, still non-zero
                }
            }
        }
    }



}




void LaneFollow::birdsEye(const cv::Mat& source, cv::Mat& destination)
{
    using namespace cv;

    double alpha        =(config_.alpha -90) * M_PI/180;
    double beta         =(config_.beta -90) * M_PI/180;
    double gamma        =(config_.gamma -90) * M_PI/180;
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


/**
 * Get robot velocity based on lane finding
 * ==========================
 *
 */
std::pair<float, float> LaneFollow::getVel(const cv::Mat& src, cv::Mat& preview)
{
    // Convert the source to grayscale
    cv::Mat mat;

    src.copyTo(mat);

    // Blur the image to reduce noise (kernel must be odd)
    if (config_.use_median_blur)
    {
        cv::medianBlur(mat, mat, 2*config_.median_blur_amount + 1);
    }

    colorMask(mat);

    // 20% fom the bottom of the image
    int row         = mat.rows - (mat.rows * 0.2);
    int row_fixed   = mat.rows - (mat.rows * 0.1);

    int ccol        = mat.cols / 2;
    int col_fixed   = mat.cols - (mat.cols * config_.fixed_center);

    int right = mat.cols;
    int left = 0;

    // Find the first white pixel on the right
    cv::Mat channels[3];
    cv::split(mat, channels);

    const cv::Mat& gray = channels[2];

    for(int i=ccol; i < mat.cols; i++)
    {
        if (gray.at<uchar>(row, i) > 10)
        {
            right = i;
            break;
        }
    }

    for (int i=ccol; i >= 0; i--)
    {
        if (gray.at<uchar>(row, i) > 10)
        {
            left = i;
            break;
        }
    }

    int center = ((right - left) / 2) + left;

    cv::line(mat,   cv::Point(0, row), cv::Point(mat.cols-1, row), cv::Scalar{40, 70, 0}, 1);
    cv::circle(mat, cv::Point(right, row),   4, cv::Scalar{100, 100, 100});
    cv::circle(mat, cv::Point(left, row),    4, cv::Scalar{100, 100, 100});
    cv::circle(mat, cv::Point(center, row),  8, cv::Scalar{100, 100, 100});
    cv::circle(mat, cv::Point(col_fixed, row_fixed),  8, cv::Scalar{60, 60, 60});

    cv::line(mat, cv::Point(center, row), cv::Point(col_fixed, row_fixed), cv::Scalar(40, 70, 0), 1);

    mat.copyTo(preview);

    int dist_from_center = col_fixed - center;
    float turn = (double)dist_from_center * (1.0 / (double)mat.cols) * 2.0;

    float speed = -0.7f;
    return std::make_pair(speed, turn * 0.7);
}

void LaneFollow::scanCb(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
}

void LaneFollow::colorMask(const cv::Mat &mat)
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

    // Create a LaneFollow object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    LaneFollow sd{};

    ROS_INFO_STREAM("birds_eye running!");
    ros::spin();
    return 0;
}
