#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace kernel {
	namespace alg {
		void normal_estimation(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			pcl::PointCloud<pcl::Normal>::Ptr normals, 
			int k);

		void normal_reorientation(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			pcl::PointCloud<pcl::Normal>::Ptr normals, 
			int k);

		void curvature_estimation(
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, 
			pcl::PointCloud<pcl::Normal>::Ptr normals, 
			int k);
	}
}