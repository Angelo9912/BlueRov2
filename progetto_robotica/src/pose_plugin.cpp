#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like WorldPlugin()
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- WorldPtr
#include <ignition/math/Pose3.hh>          // for accessing -- ignition math Pose3d()
#include <geometry_msgs/Pose.h>            // for accessing -- geometry_msgs Pose()
#include <geometry_msgs/Point.h>           // for accessing -- geometry_msgs Point()
#include <geometry_msgs/Quaternion.h>      // for accessing -- geometry_msgs Quaternion()
#include <thread>                          // for thread
#include "ros/ros.h"                       // for ros
#include "ros/callback_queue.h"            // for ros callback queue
#include "ros/subscribe_options.h"         // for ros subscribe options
#include <tf2/LinearMath/Quaternion.h>     // for tf2 quaternion
#include <tf2/LinearMath/Matrix3x3.h>      // for tf2 matrix3x3
#include "progetto_robotica/Floats.h"
#include <vector>


namespace gazebo {
    class PosePlugin : public WorldPlugin {

      private:
            double x=1, y=0, z=0, roll=0, pitch=0, yaw=0;
        
      ///A node use for ROS transport
      private: std::unique_ptr<ros::NodeHandle> rosNode;

      ///A ROS subscriber
      private: ros::Subscriber rosSub;

      ///A ROS callbackqueue that helps process messages
      private: ros::CallbackQueue rosQueue;

      ///A thread the keeps running the rosQueue
      private: std::thread rosQueueThread;

        // set publisher
      private: 
        transport::PublisherPtr publisher;

            // create msg obj
      private: 
        msgs::Model out_msg;
      
      public:
        PosePlugin() : WorldPlugin() {
            printf("Pose Plugin Created!\n");
        }

      public:
        void OnRosMsg(const progetto_robotica::FloatsConstPtr& _msg) {
            this->x = _msg->data[0];
            this->y = _msg->data[1];
            this->z = _msg->data[2];
            this->roll = 0;
            this->pitch = 0;
            this->yaw = _msg->data[3];
            
            // // set model pose
            msgs::Set(this->out_msg.mutable_pose(), ignition::math::Pose3d(this->x, this->y, this->z, this->roll, this->pitch, this->yaw));

            // Send the message
            this->publisher->Publish(this->out_msg);
        }

        ///ROS helper function that processes messages
      private: 
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }
        
      public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

            //_world pointer can access the world name using Name() fn
            std::cout << "World name = " << _world->Name() << std::endl;

            // set a node to publish
            transport::NodePtr node(new transport::Node());
            node->Init(_world->Name());


            // model to use
            this->out_msg.set_name("bluerov2");

            // set publisher
            this->publisher = node->Advertise<msgs::Model>("~/model/modify");

            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<progetto_robotica::Floats>(
                "/state_topic",
                1,
                boost::bind(&PosePlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            
            // Spin up the queue helper thread.
            this->rosQueueThread =
            std::thread(std::bind(&PosePlugin::QueueThread, this));

            // Wait for a second.
            ros::spinOnce();
        
        }        
    };

    // Register plugin 
    GZ_REGISTER_WORLD_PLUGIN(PosePlugin)
}
