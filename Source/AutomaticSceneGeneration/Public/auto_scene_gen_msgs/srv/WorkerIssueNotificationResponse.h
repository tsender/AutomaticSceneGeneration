#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// Once the ASG client receives the WorkerIssueNotification, it will submit a brief response acknowledging receipt
		class FWorkerIssueNotificationResponse : public FROSBaseServiceResponse {

		public:
			FWorkerIssueNotificationResponse() = default;
			~FWorkerIssueNotificationResponse() = default;

			// Indicates the ASG client received the notification
			bool received;
		};
	}
}