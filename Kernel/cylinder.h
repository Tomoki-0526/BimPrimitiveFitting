#pragma once

#include "surface.h"

namespace kernel {
	namespace geom {
		class cylinder : public surface
		{
		protected:
			float radius;

		public:
			cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius);
		
			auto set_radius(float radius) -> void;
			auto get_radius() const -> float;
		};
	}
}

