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

			float dissimilarity(const cylinder& other) const;
			void merge_cloud(const cylinder& other);
			int get_degrees(const Eigen::Vector3f& p) const;
		};
	}
}

