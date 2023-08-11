#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/std_msgs/ColorRGBA.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// Message for specifying scene capture settings
		class SceneCaptureSettings: public FROSBaseMsg {
		public:
			SceneCaptureSettings()
            {
                _MessageType = "auto_scene_gen_msgs/SceneCaptureSettings";
			}

			// The image size in pixels. All images will be square.
			uint32 image_size;

			// Indicates if the scene captures should contain annotations
			bool draw_annotations;

			// Goal sphere thickness in [cm]
			float goal_sphere_thickness;

			// Goal sphere color
			std_msgs::ColorRGBA goal_sphere_color;

			// Draw orthographic aerial view
			bool ortho_aerial;

			// Draw perspective aerial view
			bool perspective_aerial;

			// Padding in [m] to apply to the base orthographic width
			TArray<uint32> aerial_padding;

			// Perspective aerial-like views of the various sides/corners of the landscape. All views are from the outside looking in.
			bool front_aerial;
			bool left_front_aerial;
			bool left_aerial;
			bool left_rear_aerial;
			bool rear_aerial;
			bool right_rear_aerial;
			bool right_aerial;
			bool right_front_aerial;

			// Draw perspective view of the vehicle's POV at its starting location
			bool vehicle_start_pov;

			// Draw perspective 3rd person rear aerial view of the vehicle at the starting location
			bool vehicle_start_rear_aerial;
		};
	}
}
