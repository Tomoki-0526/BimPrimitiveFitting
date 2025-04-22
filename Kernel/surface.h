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
			pcl::PointXYZ min_pt;
			pcl::PointXYZ max_pt;

			bool valid;

		public:
			surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis);
			virtual ~surface() = 0;

			auto is_valid() const -> bool;
			auto set_valid(bool valid) -> void;
			auto merge_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> void;
			auto get_cloud() const -> pcl::PointCloud<pcl::PointXYZ>::Ptr;

			virtual auto overlap() -> void;
			virtual auto serialize() -> void;
		};
	}
}
