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

bool kernel::geom::surface::is_valid() const
{
	return this->valid;
}

void kernel::geom::surface::set_valid(bool valid)
{
	this->valid = valid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr kernel::geom::surface::get_cloud() const
{
	return this->cloud;
}
