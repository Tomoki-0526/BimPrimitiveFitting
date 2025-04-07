#pragma once

#include <filesystem>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace kernel {
	namespace io {
		class args {
		public:
			fs::path input_file;
			fs::path output_dir;

			args(int argc, char** argv);
		};

		// point cloud data
		auto load_cloud(const fs::path& path) -> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		auto load_txt_cloud(
			const fs::path& path,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			char delim = ' '
		) -> void;

		auto save_txt_cloud(
			const fs::path& path,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			const char delim = ' '
		) -> void;

		// json
		auto save_json(
			const fs::path& path,
			const nlohmann::json& obj
		) -> void;
	}
}