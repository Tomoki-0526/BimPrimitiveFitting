#pragma once

#define DEBUG_ARGUMENTS
//#define DEBUG_REMOVE_PLANAR
#define DEBUG_CYLINDER_FIT

namespace kernel {
	enum bim_category {
		none,
		wall,
		column,
		railing,
		curtainwall,
		pipe,
	};

	enum primitive_type
	{
		plane,
		sphere,
		cylinder,
		cone,
		torus,
	};

	// cloud attributes
	extern float	base_elev;
	extern float	top_elev;
	extern float	floor_height;

}