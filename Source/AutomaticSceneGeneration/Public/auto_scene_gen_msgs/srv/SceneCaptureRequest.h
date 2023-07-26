#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This services is meant for the ASG worker to pass the scene captures to the client that requested them
		class AUTOMATICSCENEGENERATION_API FSceneCaptureRequest : public FROSBaseServiceRequest {

		public:
			FSceneCaptureRequest() = default;
			~FSceneCaptureRequest() = default;

			// AutoSceneGen worker ID sending the scene captures
			uint8 worker_id;

			// Scenario number corresponding to the scene captures
			int32 scenario_number;

			// List of all scene captures (in same order as list of names)
			TArray<sensor_msgs::Image> scene_captures;

			// List of all scene capture names (in same order as list of images)
			TArray<FString> scene_capture_names;
		};
	}
}