#include "ransac.h"

#include <limits>

#include <CGAL/property_map.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

using CGALKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGALPoint = std::pair<CGALKernel::Point_3, CGALKernel::Vector_3>;
using CGALCloud = std::vector<CGALPoint>;
using PointMap = CGAL::First_of_pair_property_map<CGALPoint>;
using NormalMap = CGAL::Second_of_pair_property_map<CGALPoint>;

using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<CGALKernel, CGALCloud, PointMap, NormalMap>;
using EfficientRansac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using CGALPlane = CGAL::Shape_detection::Plane<Traits>;
using CGALCylinder = CGAL::Shape_detection::Cylinder<Traits>;
using CGALSphere = CGAL::Shape_detection::Sphere<Traits>;
using CGALCone = CGAL::Shape_detection::Cone<Traits>;
using CGALTorus = CGAL::Shape_detection::Torus<Traits>;

kernel::alg::ransac_config::ransac_config(bim_category bim_type, float eps)
	: bim_type(bim_type)
	, epsilon(eps)
	, min_pts_num(500)
	, max_nor_dev(25.0f)
	, cyl_min_radius(0.0f)
	, cyl_max_radius(std::numeric_limits<float>::max())
	, tor_min_minor_radius(0.0f)
	, tor_min_major_radius(0.0f)
	, tor_max_minor_radius(std::numeric_limits<float>::max())
	, tor_max_major_radius(std::numeric_limits<float>::max())
{
	switch (bim_type) {
	case bim_category::wall: {

	}break;
	case bim_category::column: {

	}break;
	case bim_category::curtainwall: {

	}break;
	case bim_category::pipe: {

	}break;
	case bim_category::none: {

	}break;
	default: {

	}break;
	}
}

bool kernel::alg::ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, const pcl::PointCloud<pcl::Normal>::Ptr normals, std::initializer_list<primitive_type> prim_types)
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

	ransac.detect();
}

