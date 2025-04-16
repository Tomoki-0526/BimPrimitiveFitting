#pragma once

#define DEBUG_ARGUMENTS
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

	// clustering parameters
	const  int		euc_clu_min_pts = 100;
	const  int		euc_clu_max_pts = 100000;
	const  float	euc_radius = 0.5f;
}