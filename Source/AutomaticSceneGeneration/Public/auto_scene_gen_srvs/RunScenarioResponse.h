#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages {
	namespace auto_scene_gen_srvs {
		// Once the ASG worker receives the RunScenario request, it will submit a brief response acknowledging receipt of the request
		class FRunScenarioResponse : public FROSBaseServiceResponse {

		public:
			FRunScenarioResponse() = default;
			~FRunScenarioResponse() = default;

			// Indicates the ASG worker received the RunScenario request
			bool received;
		};
	}
}