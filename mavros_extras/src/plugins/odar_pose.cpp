#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {

	class OdarPosePlugin : public MavRosPlugin {
		public:
			OdarPosePlugin() :
				nh("~odar"),
				uas(nullptr)
		{ };

			void initialize(UAS &uas_)
			{
				uas = &uas_;
				odar_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
				odar_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel", 10);
			};

			const message_map get_rx_handlers() {
				return {
	
	                		MESSAGE_HANDLER(MAVLINK_MSG_ID_ODAR_POSE, &OdarPosePlugin::handle_odar_pose)

				};
			}

		private:
			ros::NodeHandle nh;
			UAS *uas;

			ros::Publisher odar_pose_pub;
			ros::Publisher odar_vel_pub;

			void handle_odar_pose(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
				mavlink_odar_pose_t odar_pose;
				mavlink_msg_odar_pose_decode(msg, &odar_pose);

				auto pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();
				auto vel_msg = boost::make_shared<geometry_msgs::TwistStamped>();

				uint32_t sec, ns;
				sec = odar_pose.time_usec/1000000;
				ns = (odar_pose.time_usec - sec*1000000)*1000;
				pose_msg->header.stamp = ros::Time(sec, ns);
				vel_msg->header.stamp = ros::Time(sec, ns);

				pose_msg->pose.position.x = odar_pose.x; 
				pose_msg->pose.position.y = odar_pose.y; 
				pose_msg->pose.position.z = odar_pose.z; 
				pose_msg->pose.orientation.w = odar_pose.qw; 
				pose_msg->pose.orientation.x = odar_pose.qx; 
				pose_msg->pose.orientation.y = odar_pose.qy; 
				pose_msg->pose.orientation.z = odar_pose.qz; 

				vel_msg->twist.linear.x = odar_pose.vx;
				vel_msg->twist.linear.y = odar_pose.vy;
				vel_msg->twist.linear.z = odar_pose.vz;

				vel_msg->twist.angular.x = odar_pose.wx;
				vel_msg->twist.angular.y = odar_pose.wy;
				vel_msg->twist.angular.z = odar_pose.wz;

				odar_pose_pub.publish(pose_msg);
				odar_vel_pub.publish(vel_msg);
			}
	};
};

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarPosePlugin, mavplugin::MavRosPlugin)
