#pragma once

#include "global.h"

#include <initializer_list>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <CGAL/property_map.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel		CGALKernel;
typedef std::pair<CGALKernel::Point_3, CGALKernel::Vector_3>	CGALPoint;
typedef std::vector<CGALPoint>									CGALCloud;
typedef CGAL::First_of_pair_property_map<CGALPoint>				Point_map;
typedef CGAL::Second_of_pair_property_map<CGALPoint>			Normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
<CGALKernel, CGALCloud, Point_map, Normal_map>			Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> EfficientRansac;
typedef CGAL::Shape_detection::Cone<Traits>				CGALCone;
typedef CGAL::Shape_detection::Cylinder<Traits>			CGALCylinder;
typedef CGAL::Shape_detection::Plane<Traits>			CGALPlane;
typedef CGAL::Shape_detection::Sphere<Traits>			CGALSphere;
typedef CGAL::Shape_detection::Torus<Traits>			CGALTorus;

namespace kernel {
	namespace alg {
		EfficientRansac::Parameters get_ransac_params(
			float epsilon, 
			int min_points = 500,
			float deg_deviation = 25
		);

		EfficientRansac::Shape_range ransac(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			const pcl::PointCloud<pcl::Normal>::Ptr normals, 
			std::initializer_list<primitive_type> prim_types,
			const EfficientRansac::Parameters& params
		);
	}
}