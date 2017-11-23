#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OdarInfo.h>

namespace mavplugin {
/**
 * @brief OdarInfo plugin.
 */
class OdarInfoPlugin : public MavRosPlugin {
public:
	OdarInfoPlugin() :
		nh("~odar"), uas(nullptr)
	{ }

    /**
     * Plugin initializer. Constructor should not do this.
     */
	void initialize(UAS &uas_)
	{
	        uas = &uas_;
        	odar_info_pub = nh.advertise<mavros_msgs::OdarInfo>("info", 10);
	}

	const message_map get_rx_handlers() {
        	return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_ODAR_INFO, &OdarInfoPlugin::handle_odar_info),
	};
}

private:
	ros::NodeHandle nh;
	UAS *uas;


	ros::Publisher odar_info_pub;

void handle_odar_info(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	mavlink_odar_info_t odar_info;
	mavlink_msg_odar_info_decode(msg, &odar_info);

	auto ros_msg = boost::make_shared<mavros_msgs::OdarInfo>();

	uint32_t sec, ns;
	sec = odar_info.time_usec/1000000;
	ns = (odar_info.time_usec - sec*1000000)*1000;
	ros_msg->header.stamp = ros::Time(sec, ns);

	ros_msg->gain_scale = odar_info.gain_scale; 
	ros_msg->fuse_p = odar_info.fuse_flag & (1 << 0);
	ros_msg->fuse_v = odar_info.fuse_flag & (1 << 1);
	ros_msg->fuse_q = odar_info.fuse_flag & (1 << 2);
	ros_msg->fuse_w = odar_info.fuse_flag & (1 << 3);

	ros_msg->gravity_on = odar_info.feedback_flag & (1 << 0);
	ros_msg->feedback_p = odar_info.feedback_flag & (1 << 1);
	ros_msg->feedback_d = odar_info.feedback_flag & (1 << 2);
	ros_msg->feedback_i = odar_info.feedback_flag & (1 << 3);
	ros_msg->feedback_r = odar_info.feedback_flag & (1 << 4);
	ros_msg->feedback_w = odar_info.feedback_flag & (1 << 5);
	ros_msg->feedback_ri = odar_info.feedback_flag & (1 << 6);

	ros_msg->k_p = odar_info.k_p;
	ros_msg->k_d = odar_info.k_d;
	ros_msg->k_i = odar_info.k_i;
	ros_msg->k_r = odar_info.k_r;
	ros_msg->k_w = odar_info.k_w;
	ros_msg->k_ri = odar_info.k_ri;

	odar_info_pub.publish(ros_msg);
    }
};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarInfoPlugin, mavplugin::MavRosPlugin)
