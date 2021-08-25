/**
 * @brief MagCalStatus plugin
 * @file MagCalStatus.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @example MagCalStatus.cpp
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/MagnetometerReporter.h>
namespace mavros {
namespace std_plugins {

/**
 * @brief MagCalStatus plugin.
 *
 * Example and "how to" for users.
 */
class MagCalStatusPlugin : public plugin::PluginBase {
public:
	MagCalStatusPlugin() : PluginBase(),
		mcs_nh("~MagCalibration")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		mcs_pub = mcs_nh.advertise<std_msgs::UInt8>("status", 2, true);
		mcr_pub = mcs_nh.advertise<mavros_msgs::MagnetometerReporter>("report", 2, true);
	}

	/**
	 * This function returns message subscriptions.
	 *
	 * Each subscription made by PluginBase::make_handler() template.
	 * Two variations:
	 *  - With automatic decoding and framing error filtering (see handle_heartbeat)
	 *  - Raw message with framig status (see handle_systemtext)
	 */
	Subscriptions get_subscriptions() {
		return {
			/* automatic message deduction by second argument */
			make_handler(&MagCalStatusPlugin::handle_status),
            make_handler(&MagCalStatusPlugin::handle_report),
		};
	}

private:
	ros::NodeHandle mcs_nh;
	ros::Publisher mcs_pub;
    ros::Publisher mcr_pub;
	uint8_t _rgCompassCalProgress[3] = {0};
	bool calibration=true;
    //Send progress of magnetometer calibration
	void handle_status(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS &mp) {
		ROS_INFO_STREAM_NAMED("MagCalStatus", "MagCalStatus::handle_heartbeat: " << mp.to_yaml());
		auto mcs = boost::make_shared<std_msgs::UInt8>();

		// How many compasses are we calibrating?
		int compassCalCount = 0;
		for (int i=0; i<3; i++) {
			if (mp.cal_mask & (1 << i)) {
				compassCalCount++;
			}
		}

		if (mp.compass_id < 3 && compassCalCount != 0) {
			// Each compass gets a portion of the overall progress
			_rgCompassCalProgress[mp.compass_id] = mp.completion_pct / compassCalCount;
		}

		mcs->data = (_rgCompassCalProgress[0] + _rgCompassCalProgress[1] + _rgCompassCalProgress[2]);

		mcs_pub.publish(mcs);
	}
	//Send report after calibration is done
    void handle_report(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MAG_CAL_REPORT &mr) {
        ROS_INFO_STREAM_NAMED("MagCalReport", "MagCalReport:: " << mr.to_yaml());
       if(calibration) {
               auto     mcr = boost::make_shared<mavros_msgs::MagnetometerReporter>();
               mcr->report = mr.cal_status;
               mcr->confidence =mr.orientation_confidence;
               mcr_pub.publish(mcr);
               calibration=false;
       }
	}
};

}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MagCalStatusPlugin, mavros::plugin::PluginBase)
