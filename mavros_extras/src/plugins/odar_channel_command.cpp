#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <mavros_msgs/OdarChannelCommand.h>

namespace mavplugin {

	class OdarChannelCommandPlugin : public MavRosPlugin {
		public:
			OdarChannelCommandPlugin() :
				nh("~odar"),
				uas(nullptr)
		{ };

			void initialize(UAS &uas_)
			{
				uas = &uas_;
				odar_channel_command_sub = nh.subscribe("command", 10, &OdarChannelCommandPlugin::odar_channel_command_cb, this);
			};

			const message_map get_rx_handlers() {
				return {/* RX disabled */ };
			}

		private:
			ros::NodeHandle nh;
			UAS *uas;
			ros::Subscriber odar_channel_command_sub;

			void odar_channel_command_cb(const mavros_msgs::OdarChannelCommand::ConstPtr &req)
			{

				mavlink_message_t msg;
				mavlink_msg_odar_channel_command_pack_chan(UAS_PACK_CHAN(uas), &msg, 
					req->header.stamp.toNSec() / 1000,
					req->channel,
					req->float_value,
					req->int_value);
				UAS_FCU(uas)->send_message(&msg);

		

			}
	};
};

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarChannelCommandPlugin, mavplugin::MavRosPlugin)
