#include <filesystem>
#include <format>
#include <iostream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>

#include "Kernel/global.h"
#include "Kernel/io.h"
#include "Kernel/normal.h"
#include "Kernel/ransac.h"
#include "Kernel/utils.h"
#include "Kernel/vis.h"

#include "wall.h"

namespace fs = std::filesystem;

// cloud attributes
float kernel::base_elev = 0;
float kernel::top_elev = 0;
float kernel::floor_height = 0;

// shape config
float kernel::alg::epsilon = 0.1;
int kernel::alg::min_points = 500;
float kernel::alg::deg_deviation = 5.0f;
float kernel::alg::cyl_min_r = 1.0f;
float kernel::alg::cyl_max_r = 8.0f;

auto main(int argc, char** argv) -> int
{
    std::cout << "========== Curved Wall Fitting ==========" << std::endl;

#pragma region init
	// parse arguments
	kernel::io::args args(argc, argv);
#ifdef DEBUG_ARGUMENTS
	args.input_file = "D:\\scan2bim\\ZZ-01\\wall.txt";
	args.output_dir = "D:\\scan2bim\\ZZ-01\\cgal";
#endif

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
	float dx = max_pt.x - min_pt.x;
	float dy = max_pt.y - min_pt.y;
	float dz = max_pt.z - min_pt.z;
	float scale = std::max({ dx, dy, dz });
	//kernel::alg::epsilon = 0.005f * scale;

	std::cout << std::format("[Wall attributes] base elevation: {}, top elevation: {}, floor height: {}", kernel::base_elev, kernel::top_elev, kernel::floor_height) << std::endl;
#pragma endregion

#pragma region ransac
	// remove planar points
	std::cout << "Removing planar points..." << std::endl;
	auto non_planar_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	auto non_planar_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

	auto ransac_params = kernel::alg::get_ransac_params(
		kernel::alg::epsilon,
		kernel::alg::min_points, 
		kernel::alg::deg_deviation,
		kernel::alg::cyl_min_r,
		kernel::alg::cyl_max_r
	);
	Pwn_vector cgal_cloud;
	for (size_t i = 0; i < xyz->size(); ++i)
	{
		const pcl::PointXYZ& point = xyz->at(i);
		const pcl::Normal& normal = normals->at(i);
		cgal_cloud.push_back({ CGAL_kernel::Point_3(point.x, point.y, point.z), CGAL_kernel::Vector_3(normal.normal_x, normal.normal_y, normal.normal_z) });
	}
	auto shapes = kernel::alg::ransac(cgal_cloud, ransac_params, { kernel::primitive_type::plane, kernel::primitive_type::cylinder });
	for (auto it = shapes.begin(); it != shapes.end(); ++it) {
		if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
			const auto& indices = cyl->indices_of_assigned_points();
			size_t shape_size = indices.size();
			for (size_t i = 0; i < shape_size; ++i) {
				const auto& pwn = cgal_cloud[indices[i]];
				non_planar_xyz->emplace_back(pwn.first.x(), pwn.first.y(), pwn.first.z());
				non_planar_normals->emplace_back(pwn.second.x(), pwn.second.y(), pwn.second.z());
			}
		}
	}
#ifdef DEBUG_REMOVE_PLANAR
	kernel::vis::show_cloud(non_planar_xyz);
#endif

	pcl::getMinMax3D(*non_planar_xyz, min_pt, max_pt);
	dx = max_pt.x - min_pt.x;
	dy = max_pt.y - min_pt.y;
	dz = max_pt.z - min_pt.z;
	scale = std::max({ dx, dy, dz });
	//kernel::alg::epsilon = 0.005f * scale;

	// fit shapes
	std::cout << "Fitting shapes..." << std::endl;
	std::vector<bim::wall> walls;
	ransac_params = kernel::alg::get_ransac_params(
		kernel::alg::epsilon,
		kernel::alg::min_points,
		kernel::alg::deg_deviation,
		kernel::alg::cyl_min_r,
		kernel::alg::cyl_max_r
	);
	cgal_cloud.clear();
	cgal_cloud.shrink_to_fit();
	for (size_t i = 0; i < non_planar_xyz->size(); ++i)
	{
		const pcl::PointXYZ& point = non_planar_xyz->at(i);
		const pcl::Normal& normal = non_planar_normals->at(i);
		cgal_cloud.push_back({ CGAL_kernel::Point_3(point.x, point.y, point.z), CGAL_kernel::Vector_3(normal.normal_x, normal.normal_y, normal.normal_z) });
	}
	shapes = kernel::alg::ransac(cgal_cloud, ransac_params, { kernel::primitive_type::cylinder });
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> wall_clouds;
	for (auto it = shapes.begin(); it != shapes.end(); ++it) {
		if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
			std::cout << std::format("[Cylinder #{}] ", it - shapes.begin() + 1);

			const auto& p = cyl->point_on_axis();
			const auto& n = cyl->axis().to_vector();
			float r = cyl->radius();
			if (std::abs(n.z()) < 0.9) {
				std::cout << std::format("axis ({}, {}, {}) inclined.", n.x(), n.y(), n.z()) << std::endl;
				continue;
			}
			if (r < kernel::alg::cyl_min_r || r > kernel::alg::cyl_max_r) {
				std::cout << std::format("radius {} out of range: ({}, {}).", r, kernel::alg::cyl_min_r, kernel::alg::cyl_max_r) << std::endl;
				continue;
			}

			std::cout << "accepted." << std::endl;
			auto wall_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			const auto& indices = cyl->indices_of_assigned_points();
			size_t shape_size = indices.size();
			for (size_t i = 0; i < shape_size; ++i) {
				const auto& pwn = cgal_cloud[indices[i]];
				wall_cloud->emplace_back(pwn.first.x(), pwn.first.y(), pwn.first.z());
			}
			auto wall = bim::wall(wall_cloud, { float(p.x()), float(p.y()), float(p.z()) }, { float(n.x()), float(n.y()), float(n.z()) }, r);
			walls.push_back(wall);

			wall_clouds.push_back(wall_cloud);
		}
	}
#ifdef DEBUG_CYLINDER_FIT
	auto colored_cloud = kernel::vis::get_colored_cloud(wall_clouds);
	kernel::vis::show_cloud(colored_cloud);
#endif
#pragma endregion

#pragma region postprocess
	std::cout << "BIM processing..." << std::endl;

	for (int i = 0; i < walls.size(); ++i) {
		std::cout << std::format("wall #{}: ", i + 1) << std::endl;

		auto& wall = walls[i];
		if (!wall.get_arc()) {
			continue;
		}

		wall.get_segments();
	}
#pragma endregion

#pragma region output
	int valid_cnt = 0;
	for (int i = 0; i < walls.size(); ++i) {
		if (walls[i].is_valid()) {
			valid_cnt++;
			const auto& json = walls[i].serialize();
			fs::path output_path = args.output_dir / std::format("Candidate_CurvedWall_{}.json", i + 1);
			kernel::io::save_json(output_path, json);
			std::cout << std::format("Save wall to {}", output_path.string()) << std::endl;
		}
	}
	std::cout << std::format("Found {} valid walls from {} candidates.", valid_cnt, walls.size()) << std::endl;
#pragma endregion
}

