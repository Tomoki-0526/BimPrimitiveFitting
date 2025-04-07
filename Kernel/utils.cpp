#include "utils.h"

#include <pcl/filters/extract_indices.h>

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	auto extracted_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(std::make_shared<std::vector<int>>(indices));
	extractor.setNegative(false);
	extractor.filter(*extracted_cloud);

	return extracted_cloud;
}

auto kernel::utils::extract_points_by_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<size_t>& indices) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	std::vector<int> int_indices(indices.size());
	std::transform(indices.begin(), indices.end(), int_indices.begin(), [](size_t idx) { return static_cast<int>(idx); });
	return extract_points_by_indices(cloud, int_indices);
}

