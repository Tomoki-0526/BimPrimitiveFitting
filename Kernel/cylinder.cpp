#include "cylinder.h"

kernel::geom::cylinder::cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius)
	: surface(cloud, pos, axis)
	, radius(radius)
{
}

auto kernel::geom::cylinder::set_radius(float radius) -> void
{
	this->radius = radius;
}

auto kernel::geom::cylinder::get_radius() const -> float
{
	return this->radius;
}

