#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace auto_scene_gen_srvs {
	// Once the ASG worker receives the run scenario request, it will submit a brief response acknowledging receipt of the request
	class AUTOMATICSCENEGENERATION_API FRunScenarioResponse : public FROSBaseServiceResponse {

	public:
		FRunScenarioResponse() = default;
		~FRunScenarioResponse() = default;

        bool received;
	};
}
