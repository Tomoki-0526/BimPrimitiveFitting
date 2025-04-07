#include "surface.h"

kernel::geom::surface::surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis)
	: cloud(cloud)
	, pos(pos)
	, axis(axis)
	, valid(true)
{
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

auto kernel::geom::surface::get_cloud() const -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	return this->cloud;
}
