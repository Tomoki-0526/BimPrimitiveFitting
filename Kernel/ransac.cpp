#include "ransac.h"

EfficientRansac::Parameters kernel::alg::get_ransac_params(float epsilon, int min_points, float deg_deviation)
{
	EfficientRansac::Parameters params;

	params.probability = 0.01;
	params.cluster_epsilon = 0.1;

	params.min_points = min_points;
	params.epsilon = epsilon;
	params.normal_threshold = std::cosf(deg_deviation * M_PI / 180);

	return params;
}

EfficientRansac::Shape_range kernel::alg::ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, const pcl::PointCloud<pcl::Normal>::Ptr normals, std::initializer_list<primitive_type> prim_types, const EfficientRansac::Parameters& params)
{
    CGALCloud cgal_cloud;
	for (size_t i = 0; i < xyz->size(); ++i)
	{
		const pcl::PointXYZ& point = xyz->at(i);
		const pcl::Normal& normal = normals->at(i);
		cgal_cloud.push_back({ CGALKernel::Point_3(point.x, point.y, point.z), CGALKernel::Vector_3(normal.normal_x, normal.normal_y, normal.normal_z) });
	}

	EfficientRansac ransac;
	ransac.set_input(cgal_cloud);
	for (auto type : prim_types) {
		switch (type)
		{
		case primitive_type::plane:
			ransac.add_shape_factory<CGALPlane>();
			break;
		case primitive_type::cylinder:
			ransac.add_shape_factory<CGALCylinder>();
			break;
		case primitive_type::sphere:
			ransac.add_shape_factory<CGALSphere>();
			break;
		case primitive_type::cone:
			ransac.add_shape_factory<CGALCone>();
			break;
		case primitive_type::torus:
			ransac.add_shape_factory<CGALTorus>();
			break;
		}
	}
	ransac.detect(params);

	return ransac.shapes();
}

