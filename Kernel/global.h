#pragma once

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
}