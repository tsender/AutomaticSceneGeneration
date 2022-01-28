#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/nav_msgs/Path.h"

namespace ROSMessages{
	namespace auto_scene_gen_srvs {
		// The ASG worker gathers relevant data about the scenario it just ran and then submits an AnalyzeScenario request to the ASG client for analysis
		class AUTOMATICSCENEGENERATION_API FAnalyzeScenarioRequest : public FROSBaseServiceRequest {

		public:
			FAnalyzeScenarioRequest() = default;
			~FAnalyzeScenarioRequest() = default;

			uint8 worker_id;
			int32 scenario_number;
			bool crashed;
			bool succeeded;
			ROSMessages::nav_msgs::Path vehicle_path;
		};
	}
}