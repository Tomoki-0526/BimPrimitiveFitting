#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

namespace kernel {
	namespace vis {
		// display
		void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void show_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

		void show_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

		void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals);

		// color
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_colored_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, const std::vector<pcl::PointIndices>& clusters_indices);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_colored_cloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);
	}
}