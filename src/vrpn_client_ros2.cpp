/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "vrpn_client_ros2/vrpn_client_ros2.h"

#include <rclcpp/time.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <string>

namespace
{
  std::unordered_set<std::string> name_blacklist_({"VRPN Control"});
}

namespace vrpn_client_ros2
{

  /**
   * check Ros Names as defined here: http://wiki.ros.org/Names
   */
  bool isInvalidFirstCharInName(const char c)
  {
    return !(isalpha(c) || c == '/' || c == '~');
  }

  bool isInvalidSubsequentCharInName(const char c)
  {
    return !(isalnum(c) || c == '/' || c == '_');
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, rclcpp::Node::SharedPtr nh_)
  {
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());

    std::string clean_name = tracker_name;

    if (clean_name.size() > 0)
    {
      int start_subsequent = 1;
      if (isInvalidFirstCharInName(clean_name[0]))
      {
        clean_name = clean_name.substr(1);
        start_subsequent = 0;
      }

      clean_name.erase(std::remove_if(clean_name.begin() + start_subsequent, clean_name.end(), isInvalidSubsequentCharInName), clean_name.end());
    }

    init(clean_name, nh_, false);
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, std::string host, rclcpp::Node::SharedPtr nh_)
  {
    std::string tracker_address;
    tracker_address = tracker_name + "@" + host;
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_address.c_str());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    init(tracker_name, nh_, true);
  }

  void VrpnTrackerRos::init(std::string tracker_name, rclcpp::Node::SharedPtr nh, bool create_mainloop_timer)
  {
    RCLCPP_INFO_STREAM(nh->get_logger(), "Creating new tracker " << tracker_name);

    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
    tracker_remote_->shutup = true;

    std::string error;
    // TODO check name of tracker
    // if (!ros::names::validate(tracker_name, error))
    // {
    //   RCLCPP_ERROR_STREAM(nh->get_logger(),"Invalid tracker name " << tracker_name << ", not creating topics : " << error);
    //   return;
    // }

    this->tracker_name = tracker_name;
    std::stringstream ss;
    ss << nh->get_name() << "/" << tracker_name;
    this->topic_name = ss.str();

    output_nh_ = nh;

    std::string frame_id;
    nh->get_parameter_or<std::string>("frame_id", frame_id, "world");
    nh->get_parameter_or<bool>("use_server_time", use_server_time_, false);
    nh->get_parameter_or<bool>("broadcast_tf", broadcast_tf_, false);
    nh->get_parameter_or<bool>("process_sensor_id", process_sensor_id_, false);

    pose_msg_.header.frame_id = twist_msg_.header.frame_id = accel_msg_.header.frame_id = transform_stamped_.header.frame_id = frame_id;

    if (create_mainloop_timer)
    {
      double update_frequency;
      nh->get_parameter_or("update_frequency", update_frequency, 100.0);
      std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0 / update_frequency);
      mainloop_timer = nh->create_wall_timer(my_timer_duration,
                                             std::bind(&VrpnTrackerRos::mainloop, this));
    }
  }

  VrpnTrackerRos::~VrpnTrackerRos()
  {
    RCLCPP_INFO_STREAM(output_nh_->get_logger(), "Destroying tracker " << transform_stamped_.child_frame_id);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);
  }

  void VrpnTrackerRos::mainloop()
  {
    tracker_remote_->mainloop();
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    std::size_t sensor_index(0);
    auto nh = tracker->output_nh_;

    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_pose.sensor);
      std::stringstream ss;
      ss << nh->get_name() << "/" << tracker->tracker_name << "/" << std::to_string(tracker_pose.sensor);
      tracker->topic_name = ss.str();
    }

    if (tracker->pose_pubs_.size() <= sensor_index)
    {
      tracker->pose_pubs_.resize(sensor_index + 1);
    }

    if (nh->get_topic_names_and_types().count("/"+tracker->topic_name + "/pose") == 0)
    {
      tracker->pose_pubs_[sensor_index] = nh->create_publisher<geometry_msgs::msg::PoseStamped>(tracker->topic_name + "/pose", 1);
    }
    pose_pub = tracker->pose_pubs_[sensor_index];
    if (nh->count_subscribers(tracker->topic_name + "/pose") > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->pose_msg_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
        tracker->pose_msg_.header.stamp.nanosec = tracker_pose.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->pose_msg_.header.stamp = rclcpp::Time();
      }

      tracker->pose_msg_.pose.position.x = tracker_pose.pos[0];
      tracker->pose_msg_.pose.position.y = tracker_pose.pos[1];
      tracker->pose_msg_.pose.position.z = tracker_pose.pos[2];

      tracker->pose_msg_.pose.orientation.x = tracker_pose.quat[0];
      tracker->pose_msg_.pose.orientation.y = tracker_pose.quat[1];
      tracker->pose_msg_.pose.orientation.z = tracker_pose.quat[2];
      tracker->pose_msg_.pose.orientation.w = tracker_pose.quat[3];
      //RCLCPP_INFO_STREAM(nh->get_logger(),"Publishing msg in topic " << tracker->topic_name);
      pose_pub->publish(tracker->pose_msg_);
    }

    if (tracker->broadcast_tf_)
    {

      if (tracker->use_server_time_)
      {
        tracker->transform_stamped_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
        tracker->transform_stamped_.header.stamp.nanosec = tracker_pose.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->transform_stamped_.header.stamp = rclcpp::Time();
      }

      if (tracker->process_sensor_id_)
      {
        tracker->transform_stamped_.child_frame_id = tracker->tracker_name + "/" + std::to_string(tracker_pose.sensor);
      }
      else
      {
        tracker->transform_stamped_.child_frame_id = tracker->tracker_name;
      }

      tracker->transform_stamped_.transform.translation.x = tracker_pose.pos[0];
      tracker->transform_stamped_.transform.translation.y = tracker_pose.pos[1];
      tracker->transform_stamped_.transform.translation.z = tracker_pose.pos[2];

      tracker->transform_stamped_.transform.rotation.x = tracker_pose.quat[0];
      tracker->transform_stamped_.transform.rotation.y = tracker_pose.quat[1];
      tracker->transform_stamped_.transform.rotation.z = tracker_pose.quat[2];
      tracker->transform_stamped_.transform.rotation.w = tracker_pose.quat[3];

      tracker->tf_broadcaster_->sendTransform(tracker->transform_stamped_);
    }
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
    std::size_t sensor_index(0);
    auto nh = tracker->output_nh_;

    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_twist.sensor);
      std::stringstream ss;
      ss << nh->get_name() << "/" << tracker->tracker_name << "/" << std::to_string(tracker_twist.sensor);
      tracker->topic_name = ss.str();
      //nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_twist.sensor));
    }

    if (tracker->twist_pubs_.size() <= sensor_index)
    {
      tracker->twist_pubs_.resize(sensor_index + 1);
    }
    

    if (nh->get_topic_names_and_types().count("/"+tracker->topic_name + "/twist") == 0)
    {
      tracker->twist_pubs_[sensor_index]= nh->create_publisher<geometry_msgs::msg::TwistStamped>(tracker->topic_name + "/twist", 1);
    }
    twist_pub = tracker->twist_pubs_[sensor_index];
    if (nh->count_subscribers(tracker->topic_name + "/twist") > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->twist_msg_.header.stamp.sec = tracker_twist.msg_time.tv_sec;
        tracker->twist_msg_.header.stamp.nanosec = tracker_twist.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->twist_msg_.header.stamp = rclcpp::Time();
      }

      tracker->twist_msg_.twist.linear.x = tracker_twist.vel[0];
      tracker->twist_msg_.twist.linear.y = tracker_twist.vel[1];
      tracker->twist_msg_.twist.linear.z = tracker_twist.vel[2];

      double roll, pitch, yaw;
      tf2::Matrix3x3 rot_mat(
          tf2::Quaternion(tracker_twist.vel_quat[0], tracker_twist.vel_quat[1], tracker_twist.vel_quat[2],
                          tracker_twist.vel_quat[3]));
      rot_mat.getRPY(roll, pitch, yaw);

      tracker->twist_msg_.twist.angular.x = roll;
      tracker->twist_msg_.twist.angular.y = pitch;
      tracker->twist_msg_.twist.angular.z = yaw;

      twist_pub->publish(tracker->twist_msg_);
    }
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub;
    std::size_t sensor_index(0);
    auto nh = tracker->output_nh_;

    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_accel.sensor);
      std::stringstream ss;
      ss << nh->get_name() << "/" << tracker->tracker_name << "/" << std::to_string(tracker_accel.sensor);
      tracker->topic_name = ss.str();
    }

    if (tracker->accel_pubs_.size() <= sensor_index)
    {
      tracker->accel_pubs_.resize(sensor_index + 1);
    }
    

    if (nh->get_topic_names_and_types().count(tracker->topic_name + "/accel") == 0)
    {
      tracker->accel_pubs_[sensor_index] = nh->create_publisher<geometry_msgs::msg::AccelStamped>(tracker->topic_name + "/accel", 1);
    }
    accel_pub = tracker->accel_pubs_[sensor_index];
    if (nh->count_subscribers(tracker->topic_name + "/accel") > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->accel_msg_.header.stamp.sec = tracker_accel.msg_time.tv_sec;
        tracker->accel_msg_.header.stamp.nanosec = tracker_accel.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->accel_msg_.header.stamp = rclcpp::Time();
      }

      tracker->accel_msg_.accel.linear.x = tracker_accel.acc[0];
      tracker->accel_msg_.accel.linear.y = tracker_accel.acc[1];
      tracker->accel_msg_.accel.linear.z = tracker_accel.acc[2];

      double roll, pitch, yaw;
      tf2::Matrix3x3 rot_mat(
          tf2::Quaternion(tracker_accel.acc_quat[0], tracker_accel.acc_quat[1], tracker_accel.acc_quat[2],
                          tracker_accel.acc_quat[3]));
      rot_mat.getRPY(roll, pitch, yaw);

      tracker->accel_msg_.accel.angular.x = roll;
      tracker->accel_msg_.accel.angular.y = pitch;
      tracker->accel_msg_.accel.angular.z = yaw;

      accel_pub->publish(tracker->accel_msg_);
    }
  }

  VrpnClientRos::VrpnClientRos() : Node("vrpn_client_node")
  {
    // Definition of the parameters with their default value
    this->declare_parameter<std::string>("server", "192.168.2.50");
    this->declare_parameter<int>("port", 3883);
    this->declare_parameter<float>("update_frequency", 240.0);
    this->declare_parameter<float>("refresh_tracker_frequency", 1.0);
    this->declare_parameter<std::vector<std::string>>("trackers", {});
    host_ = getHostStringFromParams();

    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to VRPN server at " << host_);
    connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_.c_str()));
    // Check if the connection is estabilished
    if (!connection_->connected())
    {
      RCLCPP_ERROR(this->get_logger(), "VRPN connection is not connected!");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Connection established");
    }

    double update_frequency;
    this->get_parameter("update_frequency", update_frequency);
    std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0 / update_frequency);
    mainloop_timer = this->create_wall_timer(my_timer_duration, std::bind(&VrpnClientRos::mainloop, this));

    double refresh_tracker_frequency;
    this->get_parameter("refresh_tracker_frequency", refresh_tracker_frequency);

    if (refresh_tracker_frequency > 0.0)
    {
      std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0 / refresh_tracker_frequency);
      refresh_tracker_timer_ = this->create_wall_timer(my_timer_duration,
                                                       std::bind(&VrpnClientRos::updateTrackers, this));
    }
    std::vector<std::string> param_tracker_names_;
    if (this->get_parameter("trackers", param_tracker_names_))
    {
      for (std::vector<std::string>::iterator it = param_tracker_names_.begin();
           it != param_tracker_names_.end(); ++it)
      {
        trackers_.insert(std::make_pair(*it, std::make_shared<VrpnTrackerRos>(*it, connection_, shared_from_this())));
      }
    }
  }

  std::string VrpnClientRos::getHostStringFromParams()
  {
    std::stringstream host_stream;
    std::string server;
    int port;
    this->get_parameter("server", server);
    host_stream << server;

    if (this->get_parameter("port", port))  
    {
      host_stream << ":" << port;
    }
    return host_stream.str();
  }

  void VrpnClientRos::mainloop()
  {
    
    //auto myMap = get_topic_names_and_types();
    //RCLCPP_INFO_STREAM(get_logger(),"content "<<(myMap.count("/test_topic")==1));
    connection_->mainloop();
    if (!connection_->doing_okay())
    {
      RCLCPP_WARN(this->get_logger(), "VRPN connection is not 'doing okay'");
    }
    for (TrackerMap::iterator it = trackers_.begin(); it != trackers_.end(); ++it)
    {
      it->second->mainloop();
    }
  }

  void VrpnClientRos::updateTrackers()
  {
    int i = 0;
    while (connection_->sender_name(i) != NULL)
    {
      if (trackers_.count(connection_->sender_name(i)) == 0 && name_blacklist_.count(connection_->sender_name(i)) == 0)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Found new sender: " << connection_->sender_name(i));
        trackers_.insert(std::make_pair(connection_->sender_name(i),
                                        std::make_shared<VrpnTrackerRos>(connection_->sender_name(i), connection_,
                                                                         shared_from_this())));
      }
      i++;
    }
  }
} // namespace vrpn_client_ros2
