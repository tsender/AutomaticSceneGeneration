#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/nav_msgs/Odometry.h"

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

			// Indicates if the vehicle crashed (into an SSA)
			bool crashed;

			// Incidates if the vehicle reached the goal location
			bool succeeded;

			// The vehicle's trajectory from start to goal
			TArray<ROSMessages::nav_msgs::Odometry> vehicle_trajectory;
		};
	}
}