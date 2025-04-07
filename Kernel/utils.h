#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kernel {
	namespace utils {
		// ��ȡ������ָ�������ĵ�
		auto extract_points_by_indices(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
			const std::vector<int>& indices
		) -> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		auto extract_points_by_indices(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
			const std::vector<size_t>& indices
		) -> pcl::PointCloud<pcl::PointXYZ>::Ptr;
	}
}