#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		class StatusCode: public FROSBaseMsg {
		public:
			StatusCode() : StatusCode(0) {}

			StatusCode(uint8 status_in)
            {
				status = status_in;
                _MessageType = "auto_scene_gen_msgs/StatusCode";
			}

			uint8 status;

			static const uint8 OFFLINE = 0;
			static const uint8 ONLINE_AND_READY = 1;
			static const uint8 ONLINE_AND_RUNNING = 2;
		};
	}
}
