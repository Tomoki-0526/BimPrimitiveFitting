#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kernel {
	namespace geom {
		class surface
		{
		protected:
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
			Eigen::Vector3f pos;
			Eigen::Vector3f axis;

			bool valid;

		public:
			surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis);
			virtual ~surface() = 0;

			auto is_valid() const -> bool;
			auto set_valid(bool valid) -> void;
			auto get_cloud() const -> pcl::PointCloud<pcl::PointXYZ>::Ptr;
		};
	}
}
