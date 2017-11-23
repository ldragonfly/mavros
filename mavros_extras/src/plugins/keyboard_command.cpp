#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <std_msgs/Int32.h>

namespace mavplugin {

	class KeyboardCommandPlugin : public MavRosPlugin {
		public:
			KeyboardCommandPlugin() :
				nh("~keyboard_command"),
				uas(nullptr)
		{ };

			void initialize(UAS &uas_)
			{
				uas = &uas_;
				keyboard_sub = nh.subscribe("keyboard_sub", 10, &KeyboardCommandPlugin::keyboard_cb, this);
			};

			const message_map get_rx_handlers() {
				return {/* RX disabled */ };
			}

		private:
			ros::NodeHandle nh;
			UAS *uas;
			ros::Subscriber keyboard_sub;

			void send_to_pixhawk(int32_t cmd)
			{
				mavlink_message_t msg;
				mavlink_msg_key_command_pack_chan(UAS_PACK_CHAN(uas), &msg, cmd);
				UAS_FCU(uas)->send_message(&msg);
			}

			void keyboard_cb(const std_msgs::Int32::ConstPtr &req)
			{
//				std::cout << "Got Int32 : " << req->data <<  std::endl;
				send_to_pixhawk(req->data);
			}
	};
};

PLUGINLIB_EXPORT_CLASS(mavplugin::KeyboardCommandPlugin, mavplugin::MavRosPlugin)
