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

// ransac parameters
float kernel::alg::epsilon = 0.01;
int kernel::alg::min_points = 500;
float kernel::alg::deg_deviation = 5.0f;
float kernel::alg::cyl_min_r = 1.0f;
float kernel::alg::cyl_max_r = 10.0f;

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
	kernel::alg::epsilon = 0.01f * scale;

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
	auto shapes = kernel::alg::ransac(xyz, normals, ransac_params, { kernel::primitive_type::plane, kernel::primitive_type::cylinder });
	for (auto it = shapes.begin(); it != shapes.end(); ++it) {
		if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
			auto& indices = cyl->indices_of_assigned_points();
			kernel::utils::extract_points_by_indices(xyz, indices, non_planar_xyz);
			kernel::utils::extract_points_by_indices(normals, indices, non_planar_normals);
		}
	}

	pcl::getMinMax3D(*non_planar_xyz, min_pt, max_pt);
	dx = max_pt.x - min_pt.x;
	dy = max_pt.y - min_pt.y;
	dz = max_pt.z - min_pt.z;
	scale = std::max({ dx, dy, dz });
	kernel::alg::epsilon = 0.01f * scale;

	// fit shapes
	std::cout << "Fitting shapes..." << std::endl;
	std::vector<bim_wall::wall> walls;
	ransac_params = kernel::alg::get_ransac_params(
		kernel::alg::epsilon,
		kernel::alg::min_points,
		kernel::alg::deg_deviation,
		kernel::alg::cyl_min_r,
		kernel::alg::cyl_max_r
	);
	shapes = kernel::alg::ransac(non_planar_xyz, non_planar_normals, ransac_params, { kernel::primitive_type::cylinder });
	
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
			kernel::utils::extract_points_by_indices(non_planar_xyz, cyl->indices_of_assigned_points(), wall_cloud);
			float x = p.x(), y = p.y(), z = p.z();
			float nx = n.x(), ny = n.y(), nz = n.z();
			auto wall = bim_wall::wall(wall_cloud, { x, y, z }, { nx, ny, nz }, r);
			walls.push_back(wall);

			wall_clouds.push_back(wall_cloud);
		}
	}
#ifdef DEBUG_RANSAC
	auto colored_cloud = kernel::vis::get_colored_cloud(wall_clouds);
	kernel::vis::show_cloud(colored_cloud);
#endif
#pragma endregion
}

