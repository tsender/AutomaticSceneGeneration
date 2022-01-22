#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"

namespace auto_scene_gen_msgs {
	// The ASG submits a run scenario request describing the scenario that the ASG worker should run
	class ADVERSARIALSCENEGEN_API FRunScenarioRequest : public FROSBaseServiceRequest {

	public:
		FRunScenarioRequest() = default;
		~FRunScenarioRequest() = default;

		bool done_testing;
		int32 scenario_number;
		ROSMessages::std_msgs::Float32MultiArray ssa_array; // Structural scene actor array
	};
}
