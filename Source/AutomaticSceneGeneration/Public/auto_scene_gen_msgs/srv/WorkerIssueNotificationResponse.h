#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// Once the AutoSceneGenClient receives the WorkerIssueNotification, it will submit a brief response acknowledging receipt
		class WorkerIssueNotificationResponse : public FROSBaseServiceResponse {

		public:
			WorkerIssueNotificationResponse() = default;
			~WorkerIssueNotificationResponse() = default;

			// Indicates the AutoSceneGenClient received the notification
			bool received;
		};
	}
}