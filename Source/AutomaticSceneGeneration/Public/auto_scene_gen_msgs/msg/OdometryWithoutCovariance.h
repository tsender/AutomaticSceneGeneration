#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

#include "ROSIntegration/Public/std_msgs/Header.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "ROSIntegration/Public/geometry_msgs/Twist.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// This represents an estimate of a position and velocity in free space without covariance
		// The pose in this message should be specified in the coordinate frame given by header.frame_id
		// The twist in this message should be specified in the coordinate frame given by the child_frame_id
		class OdometryWithoutCovariance : public FROSBaseMsg {
		public:
			OdometryWithoutCovariance() {
				_MessageType = "auto_scene_gen_msgs/OdometryWithoutCovariance";
			}

			// Includes the frame id of the pose parent.
			std_msgs::Header header;

			// Frame id the pose points to. The twist is in this coordinate frame.
			FString child_frame_id;

			// Estimated pose that is typically relative to a fixed world frame.
			geometry_msgs::Pose pose;

			//Estimated linear and angular velocity relative to child_frame_id.
			geometry_msgs::Twist twist;
		};
	}
}
