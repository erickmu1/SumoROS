#include <ros/ros.h>
#include <angles/angles.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/Illuminance.h>

#include <geometry_msgs/Twist.h>

/* Movement Parameters */

double attack_speed = 1.0;
double explore_speed = 0.5;
double rotation_speed = angles::from_degrees(90);

/* Thresholds */

double illuminance_threshold = 255 / 2;

/* Cache Variables */

sensor_msgs::Range range_msg;
sensor_msgs::Illuminance illuminance_msg;

/* Callbacks to cache sensor readings */

void CacheRangeSensorReading(const sensor_msgs::Range::ConstPtr& msg)
{
    range_msg = *msg;
}

void CacheIlluminanceSensorReading(const sensor_msgs::Illuminance::ConstPtr& msg)
{
    illuminance_msg = *msg;
}

/* Executes sumo competition algorithm */
geometry_msgs::Twist ComputeCmdVel()
{
    geometry_msgs::Twist cmd_vel;

    /* Compute current environmental understanding */

    bool opponent_detected = range_msg.min_range < range_msg.range && range_msg.range < range_msg.max_range;
    bool edge_detected = illuminance_msg.illuminance > illuminance_threshold;

    /* Based on environmental understanding, determine best action */

    if (edge_detected)
    {
        // Reverse since our line detector is at the front
        cmd_vel.linear.x = -explore_speed;
    }
    else if (opponent_detected)
    {
        // Charge straight into the opponent
        cmd_vel.linear.x = attack_speed;
    }
    else
    {
        // Scan the area
        cmd_vel.angular.z = rotation_speed;
    }
    
    return cmd_vel;
}

int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it. The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "sumo_algo");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function.
    * subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe. When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue. If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    ros::Subscriber range_sub = nh.subscribe("sensor/ir", 10, CacheRangeSensorReading);
    ros::Subscriber illuminance_sub = nh.subscribe("light_sensor/illuminance", 10, CacheIlluminanceSensorReading);

   /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate loop_rate(30);

   /**
    * Determine appropriate action based on sensor data
    */
   while(ros::ok())
   {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        geometry_msgs::Twist cmd_vel;
        
        // Run Algorithm
        cmd_vel = ComputeCmdVel();

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        cmd_vel_pub.publish(cmd_vel);

        /**
         * ros::spin() will enter a loop, pumping callbacks.  With this version, all
         * callbacks will be called from within this thread (the main one). ros::spin()
         * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
         * 
         * ros::spinOnce() does not enter this loop, rather executes only one iteration.
         */
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
