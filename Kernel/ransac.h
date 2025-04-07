#pragma once

#include "global.h"

#include <initializer_list>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <CGAL/property_map.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

using CGAL_kernel		= CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_with_normal = std::pair<CGAL_kernel::Point_3, CGAL_kernel::Vector_3>;
using Pwn_vector		= std::vector<Point_with_normal>;
using Point_map			= CGAL::First_of_pair_property_map<Point_with_normal>;
using Normal_map		= CGAL::Second_of_pair_property_map<Point_with_normal>;

using Traits			= CGAL::Shape_detection::Efficient_RANSAC_traits<CGAL_kernel, Pwn_vector, Point_map, Normal_map>;
using Efficient_ransac	= CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using Plane				= CGAL::Shape_detection::Plane<Traits>;
using Cylinder			= CGAL::Shape_detection::Cylinder<Traits>;
using Sphere			= CGAL::Shape_detection::Sphere<Traits>;
using Cone				= CGAL::Shape_detection::Cone<Traits>;
using Torus				= CGAL::Shape_detection::Torus<Traits>;

namespace kernel {
	namespace alg {
		extern float	epsilon;
		extern int		min_points;
		extern float	deg_deviation;
		extern float	cyl_min_r;
		extern float	cyl_max_r;
		extern float	tor_min_r;
		extern float	tor_max_r;
		extern float	tor_min_R;
		extern float	tor_max_R;

		auto get_ransac_params(
			float epsilon,
			int min_points = 500,
			float deg_deviation = 25
		) -> Efficient_ransac::Parameters;

		auto ransac(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
			const pcl::PointCloud<pcl::Normal>::Ptr normals,
			const Efficient_ransac::Parameters& params,
			std::initializer_list<primitive_type> prim_types
		) -> Efficient_ransac::Shape_range;
	}
}