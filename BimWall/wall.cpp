#include "wall.h"

#include "Kernel/clustering.h"
#include "Kernel/global.h"
#include "Kernel/math.h"
#include "Kernel/utils.h"

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

bim::wall::wall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius)
	: cylinder(cloud, pos, axis, radius)
	, zmax(0)
	, zmin(0)
	, height(0)
{
}

bool bim::wall::calc_arc()
{
	std::vector<pcl::PointIndices> cluster_indices;
	kernel::alg::dbscan(this->cloud, kernel::euc_clu_min_pts, kernel::euc_clu_max_pts, kernel::euc_radius, cluster_indices);
	if (cluster_indices.size() == 0) {
		return false;
	}

	auto extracted_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	kernel::utils::extract_points_by_indices(this->cloud, cluster_indices[0], extracted_cloud);
	
	std::vector<int> grid(360);
	for (const auto& p : extracted_cloud->points) {
		Eigen::Vector3f v(p.x - this->pos.x(), p.y - this->pos.y(), p.z - this->pos.z());
		int degs = int(kernel::alg::degrees(v.x(), v.y()));
		if (degs < 0) degs = 0;
		if (degs >= 360) degs = 359;
		grid[degs]++;
		grid[degs < 1 ? degs + 359 : degs - 1]++;
		grid[(degs + 1) % 360]++;
	}
	int range = std::numeric_limits<int>::min();
	int deg_ed = 0;
	std::vector<int> interval(2 * 360);
	for (int i = 0; i < 2 * 360; ++i) {
		if (grid[i % 360] > 0) {
			interval[i] = i == 0 ? 1 : interval[i - 1] + 1;
			if (interval[i] > range) {
				range = interval[i];
				deg_ed = i % 360;
			}
		}
	}
	if (range < 40) {
		std::cout << "The range is too narrow, skip..." << std::endl;
		return false;
	}
	else if (range > 350) {
		std::cout << "Closed curved walls are not supported now, skip..." << std::endl;
		return false;
	}
	int deg_st = deg_ed - range;
	int deg_mi = deg_ed - range / 2;
	float rad_st = float(deg_st) / 180 * M_PI;
	if (rad_st >= -2 * M_PI && rad_st < -M_PI) {
		rad_st += 2 * M_PI;
	}
	else if (rad_st > M_PI && rad_st <= 2 * M_PI) {
		rad_st -= 2 * M_PI;
	}
	float rad_ed = float(deg_ed) / 180 * M_PI;
	if (rad_ed >= -2 * M_PI && rad_ed < -M_PI) {
		rad_ed += 2 * M_PI;
	}
	else if (rad_ed > M_PI && rad_ed <= 2 * M_PI) {
		rad_ed -= 2 * M_PI;
	}
	float rad_mi = float(deg_mi) / 180 * M_PI;
	if (rad_mi >= -2 * M_PI && rad_mi < -M_PI) {
		rad_mi += 2 * M_PI;
	}
	else if (rad_mi > M_PI && rad_mi <= 2 * M_PI) {
		rad_mi -= 2 * M_PI;
	}
	this->start_point = { this->pos.x() + this->radius * std::cos(rad_st), this->pos.y() + this->radius * std::sin(rad_st), this->pos.z() };
	this->end_point = { this->pos.x() + this->radius * std::cos(rad_ed), this->pos.y() + this->radius * std::sin(rad_ed), this->pos.z() };
	this->mid_point = { this->pos.x() + this->radius * std::cos(rad_mi), this->pos.y() + this->radius * std::sin(rad_mi), this->pos.z() };
	return true;
}

void bim::wall::calc_elev_height()
{
	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*this->cloud, min_pt, max_pt);
	this->zmax = max_pt.z;
	this->zmin = min_pt.z;

	this->height = this->zmax - this->zmin;
	if (this->height > 0.5 * kernel::floor_height) {
		this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::base_elev;
		this->height = kernel::floor_height;
	}
	else {
		if (std::abs(this->zmin - kernel::base_elev) < 0.3f) {
			this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::base_elev;
		}
		else {
			this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::top_elev - this->height;
		}
	}
}

bool bim::wall::overlap(const wall& other)
{
	pcl::PointXYZ this_min_pt, this_max_pt, other_min_pt, other_max_pt;
	pcl::getMinMax3D(*this->cloud, this_min_pt, this_max_pt);
	pcl::getMinMax3D(*other.cloud, other_min_pt, other_max_pt);
	
	bool is_overlapping = 
		(this_min_pt.x <= other_max_pt.x && this_max_pt.x >= other_min_pt.x) &&
		(this_min_pt.y <= other_max_pt.y && this_max_pt.y >= other_min_pt.y) &&
		(this_min_pt.z <= other_max_pt.z && this_max_pt.z >= other_min_pt.z);
	
	if (is_overlapping) {
		float x_overlap = std::max(0.0f, std::min(this_max_pt.x, other_max_pt.x) - std::max(this_min_pt.x, other_min_pt.x));
		float y_overlap = std::max(0.0f, std::min(this_max_pt.y, other_max_pt.y) - std::max(this_min_pt.y, other_min_pt.y));
		float z_overlap = std::max(0.0f, std::min(this_max_pt.z, other_max_pt.z) - std::max(this_min_pt.z, other_min_pt.z));
		float overlap_volume = x_overlap * y_overlap * z_overlap;
		float this_volume = (this_max_pt.x - this_min_pt.x) * (this_max_pt.y - this_min_pt.y) * (this_max_pt.z - this_min_pt.z);
		float other_volume = (other_max_pt.x - other_min_pt.x) * (other_max_pt.y - other_min_pt.y) * (other_max_pt.z - other_min_pt.z);
		float iou = overlap_volume / (this_volume + other_volume - overlap_volume);

		if (iou > 0.3f) {
			std::cout << "Overlapped with existing wall, skip..." << std::endl;
			return true;
		}
	}

	return false;
}

nlohmann::json bim::wall::serialize() const
{
	nlohmann::json obj;
	obj["typeName"] = "GbpWallCircular";
	obj["Height"] = this->height;
	obj["Width"] = 0;
	obj["StartPoint"] = this->start_point;
	obj["EndPoint"] = this->end_point;
	obj["MidPoint"] = this->mid_point;
	obj["IsStructural"] = false;

	return obj;
}
