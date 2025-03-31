#pragma once

#include "surface.h"

namespace kernel {
	namespace geom {
		class torus : public surface
		{
		protected:
			float minor_radius;
			float major_radius;
			float inner_radius;

		public:
			torus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float minor_radius, float major_radius);
		};
	}
}
