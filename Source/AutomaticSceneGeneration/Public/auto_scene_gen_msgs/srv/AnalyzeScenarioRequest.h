#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "auto_scene_gen_msgs/msg/OdometryWithoutCovariance.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// AnalyzeSceneario requests are to be submitted by the AutoSceneGenWorkers. 
		// They convey useful information to the AutoSceneGenClient about the performance of the vehicle's last run 
		class AnalyzeScenarioRequest : public FROSBaseServiceRequest {

		public:
			AnalyzeScenarioRequest() = default;
			~AnalyzeScenarioRequest() = default;

			// ASG worker ID sending the request
			uint8 worker_id;

			// Scenario number that the ASG worker just ran
			int32 scenario_number;

			// Indicates if this request only contains scene capture data
			bool scene_capture_only;

			// List of all scene captures (in same order as list of names)
			TArray<sensor_msgs::Image> scene_captures;

			// List of all scene capture names (in same order as scene_captures)
			TArray<FString> scene_capture_names;

			static const uint8 REASON_SUCCESS = 0;                // Vehicle reached the goal
			static const uint8 REASON_VEHICLE_COLLISION = 1;      // (If collisions are not allowed) Vehicle crashed into a non-traversable obstacle (any form of contact counts as collision)
			static const uint8 REASON_VEHICLE_FLIPPED = 2;        // Vehicle turned or flipped over
			static const uint8 REASON_SIM_TIMEOUT = 3;            // Simulation timeout expired
			static const uint8 REASON_VEHICLE_IDLING_TIMEOUT = 4; // Vehicle idling timeout experied
			static const uint8 REASON_VEHICLE_STUCK_TIMEOUT = 5;  // Vehicle got stuck enroute, such as on a rock or collided with an obstacle and cannot recover. Note: we separate stuck from having flipped over.
			
			// Reason for ending the simulation
			uint8 termination_reason;

			// The vehicle's trajectory from start to goal
			TArray<auto_scene_gen_msgs::OdometryWithoutCovariance> vehicle_trajectory;

			// Total length of vehicle simulation time (from when the first control input was received)
			// Sometimes rosbridge (or is it UE4?) changes the timestamps and this field can be used to verify the info was sent correctly
			float vehicle_sim_time;

			// Number of control messages received by the vehicle (can be cross-checked with how many were sent)
			int32 num_vehicle_control_messages;
		};
	}
}