#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This services is to be invoked by the AutoSceneGen Worker to inform the AutoSceneGen Client of an issue
		class AUTOMATICSCENEGENERATION_API FWorkerIssueNotificationRequest : public FROSBaseServiceRequest {

		public:
			FWorkerIssueNotificationRequest() = default;
			~FWorkerIssueNotificationRequest() = default;

			// Worker ID sending the notification
			uint8 worker_id;

			static const uint8 ISSUE_ROSBRIDGE_INTERRUPTED = 0;
			static const uint8 ISSUE_PROBLEM_CREATING_SCENE = 1;

			// ID for the issue at hand
			uint8 issue_id;

			// Message string, if any message needs to be relaid to the AutoSceneGen client. Can be empty.
			FString message;
		};
	}
}