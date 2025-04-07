#include "cylinder.h"

kernel::geom::cylinder::cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius)
	: surface(cloud, pos, axis)
	, radius(radius)
{
}

auto kernel::geom::cylinder::dissimilarity(const cylinder& other) const -> float
{
	return std::abs(this->radius - other.radius) + 
		this->axis.cross(other.axis).norm() + 
		this->axis.cross((this->pos - other.pos).normalized()).norm();
}

auto kernel::geom::cylinder::merge_cloud(const cylinder& other) -> void
{
	*(this->cloud) = *(this->cloud) + *(other.cloud);
}

auto kernel::geom::cylinder::get_degrees(const Eigen::Vector3f& p) const -> int
{
	Eigen::Vector3f v = p - this->pos;
	float rad = std::atan2(v.y(), v.x());
	if (rad < 0) {
		rad += 2 * M_PI;
	}
	return int(rad * 180 / M_PI);
}
