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

bool bim::wall::get_arc()
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
	if (range < 45) {
		std::cout << "The range is too narrow, skip..." << std::endl;
		this->valid = false;
		return false;
	}
	else if (range > 350) {
		std::cout << "Closed curved walls are not supported now, skip..." << std::endl;
		this->valid = false;
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

std::shared_ptr<bim::wall> bim::wall::get_segments()
{
	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*this->cloud, min_pt, max_pt);
	this->zmax = max_pt.z;
	this->zmin = min_pt.z;

	this->height = this->zmax - this->zmin;
	if (this->height > 0.5 * kernel::floor_height) {
		this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::base_elev;
		this->height = kernel::floor_height;
		return nullptr;
	}

	const float bin_size = 0.01f;
	int grid_num = int(this->height / bin_size) + 1;
	std::vector<float> grid(grid_num);
	for (const auto& p : this->cloud->points) {
		int z = int((p.z - this->zmin) / bin_size);
		if (z < 0) z = 0;
		if (z >= grid_num) z = grid_num - 1;
		grid[z]++;
	}

	struct interval {
		int st, ed;
		int length() const { return ed - st + 1; }
	};
	std::vector<interval> intervals;
	bool in_interval = false;
	int start = 0;
	for (int i = 0; i < grid_num; ++i) {
		if (grid[i] > 0) {
			if (!in_interval) {
				in_interval = true;
				start = i;
			}
		}
		else {
			if (in_interval) {
				in_interval = false;
				if (i - start > 5) {
					intervals.push_back({ start, i - 1 });
				}
			}
		}
	}
	if (in_interval) {
		in_interval = false;
		intervals.push_back({ start, grid_num - 1 });
	}
	std::sort(intervals.begin(), intervals.end(), [](const interval& a, const interval& b) { return a.length() > b.length(); });
	if (intervals.size() >= 2) {
		auto& first = intervals[0];
		auto& second = intervals[1];
		if (first.st > second.st) {
			std::swap(first, second);
		}
		this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::base_elev;
		this->height = first.length() * bin_size;

		wall derived(this->cloud, this->pos, this->axis, this->radius);
		derived.zmin = this->zmin + first.st * bin_size;
		derived.zmax = this->zmin + first.ed * bin_size;
		derived.height = second.length() * bin_size;
		derived.start_point = this->start_point;
		derived.end_point = this->end_point;
		derived.mid_point = this->mid_point;
		derived.start_point[2] = derived.end_point[2] = derived.mid_point[2] = kernel::top_elev - derived.height;
		derived.valid = true;

		std::cout << "derived" << std::endl;
		return std::make_shared<bim::wall>(derived);
	}
	else {
		this->start_point[2] = this->end_point[2] = this->mid_point[2] = kernel::base_elev;
		this->height = kernel::floor_height;
		return nullptr;
	}
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
