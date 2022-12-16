#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This message contains information regarding the landscape description
		class LandscapeDescription: public FROSBaseMsg {
		public:
			// Defaults to 1 subdivision and no border
			LandscapeDescription()
            {
                _MessageType = "auto_scene_gen_msgs/LandscapeDescription";
				subdivisions = 1;
				border = 0.;
			}

			// The side-length in [cm] of the nominal landscape along the X and Y dimensions (this is a square landscape)
			float nominal_size;

			// The number of times the two base triangles in the nominal landscape should be subdivided. 
			// The landscape will have 2^NumSubdivisions triangles along each edge.
     		// Each vertex in the mesh will be spaced NomSize/(2^NumSubdivisions) [cm] apart in a grid.
			int32 subdivisions;

			// (Optional) Denotes the approximate length to extend the nominal landscape in [cm]. 
     		// Using this will border the nominal landscape by ceil(Border/VertexSpacing) vertices in the four XY Cartesional directions, 
			// where VertexSpacing is discussed above.
			float border;
		};
	}
}
