#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace auto_scene_gen_msgs {
	// Once the ASG receives the analyze scenario request, it will submit a brief response acknowledging receipt of the request
	class ADVERSARIALSCENEGEN_API FAnalyzeScenarioResponse : public FROSBaseServiceResponse {

	public:
		FAnalyzeScenarioResponse() = default;
		~FAnalyzeScenarioResponse() = default;

        bool received;
	};
}
