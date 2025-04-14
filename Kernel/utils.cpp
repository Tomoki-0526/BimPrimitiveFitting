#include "utils.h"

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices& indices, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) -> void
{
	auto indices_ptr = std::make_shared<pcl::PointIndices>(indices);

	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(indices_ptr);
	extractor.setNegative(false);
	extractor.filter(*out_cloud);
}

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::Normal>::Ptr& cloud, const pcl::PointIndices& indices, pcl::PointCloud<pcl::Normal>::Ptr out_cloud) -> void
{
	auto indices_ptr = std::make_shared<pcl::PointIndices>(indices);
	
	pcl::ExtractIndices<pcl::Normal> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(indices_ptr);
	extractor.setNegative(false);
	extractor.filter(*out_cloud);
}