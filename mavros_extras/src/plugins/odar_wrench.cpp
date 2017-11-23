#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OdarWrench.h>

namespace mavplugin {
/**
 * @brief OdarWrench plugin.
 */
class OdarWrenchPlugin : public MavRosPlugin {
public:
	OdarWrenchPlugin() :
        	nh("~odar"), uas(nullptr)
	{ }

    /**
     * Plugin initializer. Constructor should not do this.
     */
	void initialize(UAS &uas_)
	{
 		uas = &uas_;
		odar_wrench_pub = nh.advertise<mavros_msgs::OdarWrench>("wrench", 10);
	}

	const message_map get_rx_handlers() {
		return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_ODAR_WRENCH, &OdarWrenchPlugin::handle_odar_wrench),
	};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;


	ros::Publisher odar_wrench_pub;

	void handle_odar_wrench(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_odar_wrench_t odar_wrench;
		mavlink_msg_odar_wrench_decode(msg, &odar_wrench);

		auto ros_msg = boost::make_shared<mavros_msgs::OdarWrench>();

		uint32_t sec, ns;
		sec = odar_wrench.time_usec/1000000;
		ns = (odar_wrench.time_usec - sec*1000000)*1000;
		ros_msg->header.stamp = ros::Time(sec, ns);

		ros_msg->linear[0] = odar_wrench.Fx;
		ros_msg->linear[1] = odar_wrench.Fy;
		ros_msg->linear[2] = odar_wrench.Fz;
		ros_msg->angular[0] = odar_wrench.Tx;
		ros_msg->angular[1] = odar_wrench.Ty;
		ros_msg->angular[2] = odar_wrench.Tz;
		odar_wrench_pub.publish(ros_msg);
	}

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarWrenchPlugin, mavplugin::MavRosPlugin)
