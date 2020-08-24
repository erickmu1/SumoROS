#ifndef GAZEBO_ROS_LIGHT_SENSOR_HH
#define GAZEBO_ROS_LIGHT_SENSOR_HH

#include <ros/ros.h>

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
    class GazeboRosLight : public CameraPlugin, GazeboRosCameraUtils
    {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosLight();

    /// \brief Destructor
    public: ~GazeboRosLight();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // "Null" image subscriber callback
    private: void NullImageCallback(const sensor_msgs::ImageConstPtr& msg) { }

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

    ros::NodeHandle* _nh;
    ros::Publisher _sensorPublisher;
    ros::Subscriber _imageSubscriber;

    std::string _namespace = "";
    std::string _sensor_name = "";
    std::string _image_topic = "image_raw";

    double _fov;
    double _range;
    };
}
#endif
