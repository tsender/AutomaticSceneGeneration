#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// Once the AutoSceneGenClient receives the AnalyzeScenario request, it will submit a brief response acknowledging receipt of the request
		class AnalyzeScenarioResponse : public FROSBaseServiceResponse {

		public:
			AnalyzeScenarioResponse() = default;
			~AnalyzeScenarioResponse() = default;

			// Indicates the AutoSceneGenClient received the AnalyzeScenario request
			bool received;
		};
	}
}