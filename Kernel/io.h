#pragma once

#include <filesystem>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <nlohmann/json.hpp>

namespace kernel {
	namespace io {
		// point cloud data
		void load_txt_cloud(
			const std::filesystem::path& path, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
			char delim = ' '
		);

		void save_txt_cloud(
			const std::filesystem::path& path, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
			const char delim = ' '
		);

		// json
		void save_json(
			const std::filesystem::path& path,
			const nlohmann::json& obj
		);
	}
}