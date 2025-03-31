#include "clustering.h"

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

void kernel::alg::region_growing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normals, int min_cluster_size, int max_cluster_size, int k, float smoothness_threshold, float curvature_threshold, std::vector<pcl::PointIndices>& clusters_indices)
{
	auto reg = std::make_shared<pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>>();
	auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	reg->setMinClusterSize(min_cluster_size);
	reg->setMaxClusterSize(max_cluster_size);
	reg->setSearchMethod(tree);
	reg->setNumberOfNeighbours(k);
	reg->setInputCloud(xyz);
	reg->setInputNormals(normals);
	reg->setSmoothnessThreshold(smoothness_threshold);
	reg->setCurvatureThreshold(curvature_threshold);
	reg->extract(clusters_indices);

	std::sort(clusters_indices.begin(), clusters_indices.end(), [](const pcl::PointIndices& c1, const pcl::PointIndices& c2) {
		return c1.indices.size() > c2.indices.size();
		});
}

void kernel::alg::dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, int min_cluster_size, int max_cluster_size, float radius, std::vector<pcl::PointIndices>& clusters_indices)
{
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	ec.setClusterTolerance(radius);
	ec.setMinClusterSize(min_cluster_size);
	ec.setMaxClusterSize(max_cluster_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(xyz);
	ec.extract(clusters_indices);
}

void kernel::alg::knn(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, int k, std::vector<std::vector<int>>& indices)
{
	int n = xyz->size();

	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(xyz);

	indices.resize(n, std::vector<int>(k));
	std::vector<std::vector<float>> distances(n, std::vector<float>(k));

	for (int i = 0; i < n; ++i) {
		tree.nearestKSearch(xyz->at(i), k, indices[i], distances[i]);
	}
}
