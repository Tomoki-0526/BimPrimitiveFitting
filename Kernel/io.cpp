#include "io.h"

#include <fstream>
#include <format>

void kernel::io::load_txt_cloud(const std::filesystem::path& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char delim)
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

void kernel::io::save_txt_cloud(const std::filesystem::path& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const char delim)
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

void kernel::io::save_json(const std::filesystem::path& path, const nlohmann::json& obj)
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
