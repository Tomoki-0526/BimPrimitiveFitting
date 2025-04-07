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

			auto dissimilarity(const cylinder& other) const -> float;
			auto merge_cloud(const cylinder& other) -> void;
			auto get_degrees(const Eigen::Vector3f& p) const -> int;
		};
	}
}

