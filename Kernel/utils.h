#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

namespace kernel {
	namespace utils {
		// 提取点云中指定索引的点
		auto extract_points_by_indices(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
			const pcl::PointIndices& indices,
			pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud
		) -> void;

		auto extract_points_by_indices(
			const pcl::PointCloud<pcl::Normal>::Ptr& cloud,
			const pcl::PointIndices& indices,
			pcl::PointCloud<pcl::Normal>::Ptr out_cloud
		) -> void;
	}
}