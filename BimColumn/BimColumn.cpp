#include <iostream>

#include <pcl/common/common.h>

#include "Kernel/clustering.h"
#include "Kernel/global.h"
#include "Kernel/io.h"
#include "Kernel/normal.h"
#include "Kernel/ransac.h"
#include "Kernel/utils.h"
#include "Kernel/vis.h"

#include "column.h"

namespace fs = std::filesystem;

// cloud attributes
float kernel::base_elev = 0;
float kernel::top_elev = 0;
float kernel::floor_height = 0;

// cluster config
int kernel::alg::min_cluster_size = 100;
int kernel::alg::max_cluster_size = 10000;
float kernel::alg::cluster_radius = 0.2f;

// shape config
float kernel::alg::epsilon = 0.1;
int kernel::alg::min_points = 100;
float kernel::alg::deg_deviation = 5.0f;
float kernel::alg::cyl_min_r = 0.0f;
float kernel::alg::cyl_max_r = 2.0f;

auto main(int argc, char** argv) -> int
{
    std::cout << "========== Circular Column Fitting ==========" << std::endl;

#pragma region init
	// parse arguments
	kernel::io::args args(argc, argv);

	// load point cloud
	auto xyz = kernel::io::load_cloud(args.input_file);
	std::cout << std::format("Loaded {} points from {}", xyz->size(), args.input_file.string()) << std::endl;
	size_t size = xyz->size();
#pragma endregion

#pragma region feature calculate
	// normals
	std::cout << "Calculating normals..." << std::endl;
	auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
	kernel::alg::normal_estimate(xyz, normals, kernel::alg::normal_estimate_k);
	kernel::alg::normal_orient(xyz, normals, kernel::alg::normal_orient_k);
	// scale
	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*xyz, min_pt, max_pt);
	kernel::base_elev = min_pt.z;
	kernel::top_elev = max_pt.z;
	kernel::floor_height = max_pt.z - min_pt.z;

	std::cout << std::format("[Wall attributes] base elevation: {}, top elevation: {}, floor height: {}", kernel::base_elev, kernel::top_elev, kernel::floor_height) << std::endl;
#pragma endregion

#pragma region instance ransac fit
	std::cout << "Instances fitting..." << std::endl;

	std::vector<pcl::PointIndices> clusters_indices;
	kernel::alg::dbscan(xyz, kernel::alg::min_cluster_size, kernel::alg::max_cluster_size, kernel::alg::cluster_radius, clusters_indices);

	auto colored_cloud = kernel::vis::get_colored_cloud(xyz, clusters_indices);
	kernel::vis::show_cloud(colored_cloud);

	size_t cylinder_count = 0;
	std::vector<bim::column> columns;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> column_clouds;
	for (const auto& indices : clusters_indices) {
		auto cluster_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		auto cluster_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
		kernel::utils::extract_points_by_indices(xyz, indices, cluster_xyz);
		kernel::utils::extract_points_by_indices(normals, indices, cluster_normals);

		pcl::getMinMax3D(*xyz, min_pt, max_pt);
		float dx = max_pt.x - min_pt.x;
		float dy = max_pt.y - min_pt.y;
		float dz = max_pt.z - min_pt.z;
		float scale = std::max({ dx, dy, dz });
		kernel::alg::epsilon = 0.005 * scale;

		auto ransac_params = kernel::alg::get_ransac_params(
			kernel::alg::epsilon,
			kernel::alg::min_points,
			kernel::alg::deg_deviation,
			kernel::alg::cyl_min_r,
			kernel::alg::cyl_max_r
		);
		Pwn_vector cgal_cloud;
		for (size_t i = 0; i < cluster_xyz->size(); ++i)
		{
			const pcl::PointXYZ& point = cluster_xyz->at(i);
			const pcl::Normal& normal = cluster_normals->at(i);
			cgal_cloud.push_back({ CGAL_kernel::Point_3(point.x, point.y, point.z), CGAL_kernel::Vector_3(normal.normal_x, normal.normal_y, normal.normal_z) });
		}
		auto shapes = kernel::alg::ransac(
			cgal_cloud, 
			ransac_params, 
			{ 
				kernel::primitive_type::plane, 
				kernel::primitive_type::cylinder 
			}
		);
		auto main_shape = shapes.begin();
		if (Cylinder* cyl = dynamic_cast<Cylinder*>(main_shape->get())) {
			std::cout << std::format("[Cylinder #{}] ", cylinder_count + 1);

			const auto& p = cyl->point_on_axis();
			const auto& n = cyl->axis().to_vector();
			float r = cyl->radius();
			if (std::abs(n.z()) < 0.9) {
				std::cout << std::format("axis ({:.4f}, {:.4f}, {:.4f}) not vertical.", n.x(), n.y(), n.z()) << std::endl;
				continue;
			}
			if (r < kernel::alg::cyl_min_r || r > kernel::alg::cyl_max_r) {
				std::cout << std::format("radius {} out of range: ({}, {}).", r, kernel::alg::cyl_min_r, kernel::alg::cyl_max_r) << std::endl;
				continue;
			}

			std::cout << "accepted." << std::endl;
			auto column_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			const auto& indices = cyl->indices_of_assigned_points();
			size_t shape_size = indices.size();
			for (size_t i = 0; i < shape_size; ++i) {
				const auto& pwn = cgal_cloud[indices[i]];
				column_cloud->emplace_back(pwn.first.x(), pwn.first.y(), pwn.first.z());
			}
			auto column = bim::column(
				column_cloud, 
				{ 
					float(p.x()), 
					float(p.y()), 
					kernel::base_elev
				}, 
				{ 
					0.0,
					0.0,
					1.0
				}, 
				r,
				kernel::floor_height
			);
			columns.push_back(column);

			column_clouds.push_back(column_cloud);
		}
	}
	std::sort(columns.begin(), columns.end(), [](const bim::column& a, const bim::column& b) {
		return a.get_cloud()->size() > b.get_cloud()->size();
		});
#ifdef DEBUG_CYLINDER_FIT
	auto colored_cloud = kernel::vis::get_colored_cloud(column_clouds);
	kernel::vis::show_cloud(colored_cloud);
#endif
#pragma endregion

#pragma region postprocess
	std::cout << "BIM processing..." << std::endl;

	if (columns.empty()) {
		std::cout << "No column found." << std::endl;
		return 0;
	}

	// deduplicate
	for (int i = 0; i < columns.size(); ++i) {
		if (!columns[i].is_valid()) {
			continue;
		}
		for (int j = i + 1; j < columns.size(); ++j) {
			if (columns[j].is_valid() && columns[i].overlap(columns[j])) {
				columns[i].merge_cloud(columns[j].get_cloud());
				columns[j].set_valid(false);
			}
		}
	}

	// radius
	float avg_radius = 0;
	for (const auto& column : columns) {
		if (column.is_valid()) {
			avg_radius += column.get_radius();
		}
	}
	avg_radius /= columns.size();
	for (auto& column : columns) {
		if (column.is_valid()) {
			column.set_radius(avg_radius);
		}
	}
#pragma endregion

#pragma region output
	int valid_cnt = 0;
	for (int i = 0; i < columns.size(); ++i) {
		if (columns[i].is_valid()) {
			valid_cnt++;
			const auto& json = columns[i].serialize();
			fs::path output_path = args.output_dir / std::format("Candidate_CircularColumn_{}.json", i + 1);
			kernel::io::save_json(output_path, json);
			std::cout << std::format("Save column to {}", output_path.string()) << std::endl;
		}
	}
	std::cout << std::format("Found {} valid walls from {} candidates.", valid_cnt, columns.size()) << std::endl;
#pragma endregion

	return 0;
}

