#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace kernel {
	namespace alg {
		const int		normal_estimate_k = 6;
		const int		normal_orient_k = 40;
		const int		curvature_estimate_k = 6;

		auto normal_estimate(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
			pcl::PointCloud<pcl::Normal>::Ptr normals,
			int k
		) -> void;

		auto normal_orient(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
			pcl::PointCloud<pcl::Normal>::Ptr normals,
			int k
		) -> void;

		auto curvature_estimate(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			pcl::PointCloud<pcl::Normal>::Ptr normals, 
			int k
		) -> void;
	}
}