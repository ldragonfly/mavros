#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OdarPWM.h>

namespace mavplugin {
/**
 * @brief OdarPWM plugin.
 */
class OdarPWMPlugin : public MavRosPlugin {
public:
	OdarPWMPlugin() :
		nh("~odar"), uas(nullptr)
	{ }

    /**
     * Plugin initializer. Constructor should not do this.
     */
	void initialize(UAS &uas_)
	{
	        uas = &uas_;
        	odar_pwm_pub = nh.advertise<mavros_msgs::OdarPWM>("pwm", 10);
	}

	const message_map get_rx_handlers() {
        	return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_ODAR_PWM, &OdarPWMPlugin::handle_odar_pwm),
	};
}

private:
	ros::NodeHandle nh;
	UAS *uas;


	ros::Publisher odar_pwm_pub;

void handle_odar_pwm(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	mavlink_odar_pwm_t odar_pwm;
	mavlink_msg_odar_pwm_decode(msg, &odar_pwm);

	auto ros_msg = boost::make_shared<mavros_msgs::OdarPWM>();

	uint32_t sec, ns;
	sec = odar_pwm.time_usec/1000000;
	ns = (odar_pwm.time_usec - sec*1000000)*1000;
	ros_msg->header.stamp = ros::Time(sec, ns);

	ros_msg->pwm[0] = odar_pwm.pwm1; 
	ros_msg->pwm[1] = odar_pwm.pwm2;
	ros_msg->pwm[2] = odar_pwm.pwm3;
	ros_msg->pwm[3] = odar_pwm.pwm4;
	ros_msg->pwm[4] = odar_pwm.pwm5;
	ros_msg->pwm[5] = odar_pwm.pwm6;
	ros_msg->pwm[6] = odar_pwm.pwm7;
	ros_msg->pwm[7] = odar_pwm.pwm8;

	odar_pwm_pub.publish(ros_msg);
    }
};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarPWMPlugin, mavplugin::MavRosPlugin)
