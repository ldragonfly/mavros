#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace mavplugin {
/**
 * @brief Vision twist estimate plugin
 *
 * Send twist estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionTwistEstimatePlugin : public MavRosPlugin,
	private TF2ListenerMixin<VisionTwistEstimatePlugin> {
public:
	VisionTwistEstimatePlugin() :
		sp_nh("~vision_twist"),
		uas(nullptr),
	{ };

	void initialize(UAS &uas_)
	{

		uas = &uas_;
		vision_sub = sp_nh.subscribe("twist", 10, &VisionTwistEstimatePlugin::vision_cb, this);
		vision_cov_sub = sp_nh.subscribe("twist_cov", 10, &VisionTwistEstimatePlugin::vision_cov_cb, this);
		}
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber vision_sub;
	ros::Subscriber vision_cov_sub;

	void send_vision_velocity_estimate(const ros::Time &stamp, const geometry_msgs::Twist &twist) {
		uint64_t usec = stamp.toNSec() / 1000;
		float vx,vy,vz,wx,wy,wz;
		vx = twist.linear.x;
		vy = twist.linear.y;
		vz = twist.linear.z;
		wx = twist.angular.x;
		wy = twist.angular.y;
		wz = twist.angular.z;

		mavlink_message_t msg;
		mavlink_msg_vision_velocity_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				vx,
				vy,
				vz,
				wx,
				wy,
				wz);
		UAS_FCU(uas)->send_message(&msg);
	}


	}

	void vision_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		geometry_msgs::Twist twist;
		twist = req->twist;
		send_vision_estimate(req->header.stamp, twist);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionTwistEstimatePlugin, mavplugin::MavRosPlugin)
