#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// Once the ASG client receives the AnalyzeScenario request, it will submit a brief response acknowledging receipt of the request
		class AUTOMATICSCENEGENERATION_API FAnalyzeScenarioResponse : public FROSBaseServiceResponse {

		public:
			FAnalyzeScenarioResponse() = default;
			~FAnalyzeScenarioResponse() = default;

			// Indicates the ASG client received the AnalyzeScenario request
			bool received;
		};
	}
}