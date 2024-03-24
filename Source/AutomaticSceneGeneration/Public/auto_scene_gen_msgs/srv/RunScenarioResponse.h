#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// Once the AutoSceneGenWorker receives the RunScenario request, it will submit a brief response acknowledging receipt of the request
		class RunScenarioResponse : public FROSBaseServiceResponse {

		public:
			RunScenarioResponse() = default;
			~RunScenarioResponse() = default;

			// Indicates the AutoSceneGenWorker received the RunScenario request
			bool received;
		};
	}
}