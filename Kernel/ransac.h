#pragma once

#include "global.h"

#include <initializer_list>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace kernel {
	namespace alg {
		struct ransac_config {
			ransac_config(bim_category bim_type, float eps = 0.1f);

			bim_category bim_type;

			float epsilon;					// max distance to primitive
			int min_pts_num;				// min support number of points for a primitive patch
			float max_nor_dev;				// max normal deviation
			float cyl_min_radius;			// min radius for cylinder
			float cyl_max_radius;			// max radius for cylinder
			float tor_min_minor_radius;		// min minor radius for torus
			float tor_min_major_radius;		// min major radius for torus
			float tor_max_minor_radius;		// max minor radius for torus
			float tor_max_major_radius;		// max major radius for torus
		};

		bool ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, const pcl::PointCloud<pcl::Normal>::Ptr normals, std::initializer_list<primitive_type> prim_types);
	}
}