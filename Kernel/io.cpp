#include "io.h"

#include <fstream>
#include <format>

#include <pcl/io/ply_io.h>

#include <argparse/argparse.hpp>

kernel::io::args::args(int argc, char** argv)
{
    argparse::ArgumentParser parser("BpfWall");
    parser.add_argument("-i", "--input")
        .required()
        .help("input point cloud file in [.ply] or [.txt] format")
        .default_value(std::string());
    parser.add_argument("-o", "--output")
        .help("output directory")
        .default_value(std::string());

    try {
        parser.parse_args(argc, argv);
    }
    catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        std::exit(1);
    }

    this->input_file = parser.get<std::string>("--input");
    if (!parser.is_used("--output")) {
        this->output_dir = parser.get<std::string>("--output");
        if (!fs::exists(output_dir)) {
            if (!fs::create_directories(output_dir)) {
                std::cerr << std::format("Failed to create directory: {}", output_dir.string()) << std::endl;
                std::exit(1);
            }
        }
    }
    else {
        this->output_dir = this->input_file.parent_path();
    }
}

auto kernel::io::load_cloud(const fs::path& path) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	auto ext = path.extension().string();
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
	try {
		if (ext == ".ply") {
			if (pcl::io::loadPLYFile(path.string(), *cloud) == -1) {
				throw std::runtime_error(std::format("Failed to load PLY file: {}", path.string()));
			}
		}
		else if (ext == ".txt") {
			load_txt_cloud(path, cloud);
		}
		else {
			throw std::runtime_error(std::format("Unsupported file format: {}", path.extension().string()));
		}
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		std::exit(1);
	}
	return cloud;
}

auto kernel::io::load_txt_cloud(const fs::path& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char delim) -> void
{
    std::ifstream ifs(path.string(), std::ios::in);
    if (!ifs) {
        throw std::runtime_error(std::format("Failed to open file: {}", path.string()));
    }
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (delim == ' ') {
            if (!(iss >> x >> y >> z)) {
                throw std::runtime_error(std::format("Failed to read line: {}", line));
            }
        }
        else {
            if (!(iss >> x >> delim >> y >> delim >> z)) {
                throw std::runtime_error(std::format("Failed to read line: {}", line));
            }
        }
        cloud->points.emplace_back(x, y, z);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    ifs.close();
}

auto kernel::io::save_txt_cloud(const fs::path& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const char delim) -> void
{
    std::ofstream ofs(path.string(), std::ios::out);
    if (!ofs) {
        throw std::runtime_error(std::format("Failed to open file: {}", path.string()));
    }
    for (const auto& p : cloud->points) {
        ofs << std::format("{}{}{}{}{}", p.x, delim, p.y, delim, p.z) << std::endl;
    }
    ofs.close();
}

auto kernel::io::save_json(const fs::path& path, const nlohmann::json& obj) -> void
{
    std::ofstream fout(path.string());
    if (fout.is_open()) {
        fout << std::setw(4) << obj << std::endl;
        fout.close();
    }
    else {
        throw std::runtime_error(std::format("Failed to open file: {}", path.string()));
    }
}
