#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		class EnableStatus: public FROSBaseMsg {
		public:
			EnableStatus() : EnableStatus(false) {}

			EnableStatus(bool enable)
            {
				enabled = enable;
                _MessageType = "auto_scene_gen_msgs/EnableStatus";
			}

			bool enabled;
		};
	}
}
