#include "surface.h"

#include <pcl/common/common.h>

kernel::geom::surface::surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis)
	: cloud(cloud)
	, pos(pos)
	, axis(axis)
	, valid(true)
{
	pcl::getMinMax3D(*this->cloud, min_pt, max_pt);
}

kernel::geom::surface::~surface()
{
}

auto kernel::geom::surface::is_valid() const -> bool
{
	return this->valid;
}

auto kernel::geom::surface::set_valid(bool valid) -> void
{
	this->valid = valid;
}

auto kernel::geom::surface::merge_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> void
{
	*this->cloud += *cloud;
}

auto kernel::geom::surface::get_cloud() const -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	return this->cloud;
}

auto kernel::geom::surface::overlap() -> void
{
}

auto kernel::geom::surface::serialize() -> void
{
}
