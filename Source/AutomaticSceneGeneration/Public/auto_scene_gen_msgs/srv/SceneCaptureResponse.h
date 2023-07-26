#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// Once the ASG client receives the SceneCaptures request, it will submit a brief response acknowledging receipt of the data
		class FSceneCaptureResponse : public FROSBaseServiceResponse {

		public:
			FSceneCaptureResponse() = default;
			~FSceneCaptureResponse() = default;

			// Indicates the ASG client received the scene captures
			bool received;
		};
	}
}