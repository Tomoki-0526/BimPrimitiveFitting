#include "utils.h"

#include <pcl/filters/extract_indices.h>

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<size_t>& indices, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) -> void
{
	std::vector<int> int_indices(indices.size());
	std::transform(indices.begin(), indices.end(), int_indices.begin(), [](size_t idx) { return static_cast<int>(idx); });

	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(std::make_shared<std::vector<int>>(int_indices));
	extractor.setNegative(false);
	extractor.filter(*out_cloud);
}

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::Normal>::Ptr& cloud, const std::vector<size_t>& indices, pcl::PointCloud<pcl::Normal>::Ptr out_cloud) -> void
{
	std::vector<int> int_indices(indices.size());
	std::transform(indices.begin(), indices.end(), int_indices.begin(), [](size_t idx) { return static_cast<int>(idx); });
	
	pcl::ExtractIndices<pcl::Normal> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(std::make_shared<std::vector<int>>(int_indices));
	extractor.setNegative(false);
	extractor.filter(*out_cloud);
}