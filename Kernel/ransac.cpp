#include "ransac.h"

auto kernel::alg::get_ransac_params(float epsilon, int min_points, float deg_deviation, float cyl_min_r, float cyl_max_r, float tor_min_r, float tor_max_r, float tor_min_R, float tor_max_R) -> Efficient_ransac::Parameters
{
	Efficient_ransac::Parameters params;

	params.probability = 0.01;
	params.cluster_epsilon = 0.1;

	params.min_points = min_points;
	params.epsilon = epsilon;
	params.normal_threshold = std::cosf(deg_deviation * M_PI / 180);

	params.cyl_min_radius = cyl_min_r;
	params.cyl_max_radius = cyl_max_r;
	params.tor_min_minor_radius = tor_min_r;
	params.tor_max_minor_radius = tor_max_r;
	params.tor_min_major_radius = tor_min_R;
	params.tor_max_major_radius = tor_max_R;

	return params;
}

auto kernel::alg::ransac(Pwn_vector& cgal_cloud, const Efficient_ransac::Parameters& params, const std::initializer_list<primitive_type>& prim_types) -> Efficient_ransac::Shape_range
{
	Efficient_ransac ransac;
	ransac.set_input(cgal_cloud);
	for (auto type : prim_types) {
		switch (type)
		{
		case primitive_type::plane:
			ransac.add_shape_factory<Plane>();
			break;
		case primitive_type::cylinder:
			ransac.add_shape_factory<Cylinder>();
			break;
		case primitive_type::sphere:
			ransac.add_shape_factory<Sphere>();
			break;
		case primitive_type::cone:
			ransac.add_shape_factory<Cone>();
			break;
		case primitive_type::torus:
			ransac.add_shape_factory<Torus>();
			break;
		}
	}
	ransac.detect(params);

	return ransac.shapes();
}

