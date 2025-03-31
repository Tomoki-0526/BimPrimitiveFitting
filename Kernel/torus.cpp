#include "torus.h"

kernel::geom::torus::torus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float minor_radius, float major_radius)
	: surface(cloud, pos, axis)
	, minor_radius(minor_radius)
	, major_radius(major_radius)
{
	this->inner_radius = this->major_radius - this->minor_radius;
}
