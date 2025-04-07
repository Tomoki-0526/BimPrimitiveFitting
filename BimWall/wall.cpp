#include "wall.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

bim_wall::wall::wall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius)
	: cylinder(cloud, pos, axis, radius)
	, zmax(0)
	, zmin(0)
	, height(0)
{
}
