#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

namespace kernel {
	namespace alg {
		auto region_growing(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			pcl::PointCloud<pcl::Normal>::Ptr normals, 
			int min_cluster_size, 
			int max_cluster_size, 
			int k, 
			float smoothness_threshold, 
			float curvature_threshold, 
			std::vector<pcl::PointIndices>& clusters_indices
		) -> void;

		auto dbscan(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
			int min_cluster_size,
			int max_cluster_size,
			float radius,
			std::vector<pcl::PointIndices>& clusters_indices
		) -> void;

		auto knn(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
			int k,
			std::vector<std::vector<int>>& indices
		) -> void;
	}
}