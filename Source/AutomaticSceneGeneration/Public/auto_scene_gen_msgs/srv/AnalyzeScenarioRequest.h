#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "auto_scene_gen_msgs/msg/OdometryWithoutCovariance.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// AnalyzeSceneario requests are to be submitted by the ASG workers. 
		// They convey useful information to the ASG client about the performance of the vehicle's last run 
		class AUTOMATICSCENEGENERATION_API FAnalyzeScenarioRequest : public FROSBaseServiceRequest {

		public:
			FAnalyzeScenarioRequest() = default;
			~FAnalyzeScenarioRequest() = default;

			// ASG worker ID sending the request
			uint8 worker_id;

			// Scenario number that the ASG worker just ran
			int32 scenario_number;

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
		};
	}
}