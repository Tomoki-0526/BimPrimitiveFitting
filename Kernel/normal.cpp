#include "normal.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/features/principal_curvatures.h>

#include <open3d/geometry/PointCloud.h>

auto kernel::alg::normal_estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normals, int k) -> void
{
	assert(k > 0);

	auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyz);
	ne.setSearchMethod(tree);
	ne.setKSearch(k);
	ne.compute(*normals);
}

auto kernel::alg::normal_orient(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normals, int k) -> void
{
	assert(k > 0);

	open3d::geometry::PointCloud cloud;
	int n = xyz->size();
	for (int i = 0; i < n; ++i) {
		cloud.points_.emplace_back(xyz->at(i).x, xyz->at(i).y, xyz->at(i).z);
		cloud.normals_.emplace_back(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
	}
	cloud.OrientNormalsConsistentTangentPlane(k);
	for (int i = 0; i < n; ++i) {
		xyz->at(i) = { static_cast<float>(cloud.points_.at(i).x()), static_cast<float>(cloud.points_.at(i).y()), static_cast<float>(cloud.points_.at(i).z()) };
		normals->at(i) = { static_cast<float>(cloud.normals_.at(i).x()), static_cast<float>(cloud.normals_.at(i).y()), static_cast<float>(cloud.normals_.at(i).z()) };
	}
}

auto kernel::alg::curvature_estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normals, int k) -> void
{
	assert(k > 0);

	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce;
	auto curvatures = std::make_shared<pcl::PointCloud<pcl::PrincipalCurvatures>>();
	auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	pce.setInputCloud(xyz);
	pce.setInputNormals(normals);
	pce.setSearchMethod(tree);
	pce.setKSearch(k);
	pce.compute(*curvatures);

	for (int i = 0; i < curvatures->size(); ++i) {
		normals->at(i).curvature = 0.5 * (curvatures->at(i).pc1 + curvatures->at(i).pc2);
	}

	pcl::PointCloud<pcl::PrincipalCurvatures> curvatures_clone = *curvatures;
	std::sort(curvatures_clone.begin(), curvatures_clone.end(), [](const pcl::PrincipalCurvatures& p1, const pcl::PrincipalCurvatures& p2) {
		return p1.pc1 + p1.pc2 < p2.pc1 + p2.pc2;
		});
}
