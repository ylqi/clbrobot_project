
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <depth_image_proc/depth_traits.h>
#include <image_transport/image_transport.h>
#include <string.h>

class ClbrobotFollower {

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_;
  ros::Publisher cmdpub_;
  cv::Mat depthimage;
private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  bool   camera_model_; /**< Enable/disable following; just prevents motor commands */


public:
  ClbrobotFollower() : min_y_(0.1), max_y_(0.5),
                   min_x_(-0.2), max_x_(0.2),
                   max_z_(0.8), goal_z_(0.6),
                   z_scale_(1.0), x_scale_(5.0),it(nh)
  { 
    ros::NodeHandle private_nh("~");

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);
    private_nh.getParam("camera_model", camera_model_);

    cmdpub_ = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    sub_= it.subscribe("depth_image", 1, &ClbrobotFollower::imagecb, this);

  }

  ~ClbrobotFollower(){ }

  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
     cv_bridge::CvImagePtr cv_ptr;
     try{
          cv_ptr = cv_bridge::toCvCopy(depth_msg,sensor_msgs::image_encodings::TYPE_32FC1);
          cv_ptr->image.copyTo(depthimage);
     }catch (cv_bridge::Exception& e){
          ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", depth_msg->encoding.c_str());
     }


    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    //ROS_INFO("image_width:%f, x_radians_per_pixel %f ", depthimage.at<float>(240, 320), x_radians_per_pixel);
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    float depth = 0.0;
    //Number of points observed
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    for (int v = 0; v < (int)depth_msg->height; ++v){
     for (int u = 0; u < (int)depth_msg->width; ++u){
       if(camera_model_)
         depth = depth_image_proc::DepthTraits<float>::toMeters(depthimage.at<float>(v, u));
       else
         depth = depth_image_proc::DepthTraits<float>::toMeters(depthimage.at<float>(v, u))/1000;

       if (depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ && x_val > min_x_ && x_val < max_x_){
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    if (n>4000){
      x /= n;
      y /= n;
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
        if (enabled_){
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);

      if (enabled_){
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        cmdpub_.publish(cmd);
      }
    }else{
      ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
      if (enabled_){
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
    }

  }

};

int main(int argc, char** argv )
{
    ros::init(argc, argv, "clbrobot_follower_node");
    ClbrobotFollower clbrobot_follower;
    ros::spin();
    return 0;
}

