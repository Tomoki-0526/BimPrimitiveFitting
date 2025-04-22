#include "column.h"

bim::column::column(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius, float height)
	: cylinder(cloud, pos, axis, radius)
	, height(height)
{
}

auto bim::column::overlap(const column& other) -> bool
{
	bool is_overlapping =
		(this->min_pt.x <= other.max_pt.x && this->max_pt.x >= other.min_pt.x) &&
		(this->min_pt.y <= other.max_pt.y && this->max_pt.y >= other.min_pt.y);

	if (is_overlapping) {
		float x_overlap = std::max(0.0f, std::min(this->max_pt.x, other.max_pt.x) - std::max(this->min_pt.x, other.min_pt.x));
		float y_overlap = std::max(0.0f, std::min(this->max_pt.y, other.max_pt.y) - std::max(this->min_pt.y, other.min_pt.y));
		float overlap_area = x_overlap * y_overlap;
		float this_area = (this->max_pt.x - this->min_pt.x) * (this->max_pt.y - this->min_pt.y);
		float other_area = (other.max_pt.x - other.min_pt.x) * (other.max_pt.y - other.min_pt.y);
		float iou = overlap_area / (this_area + other_area - overlap_area);

		if (iou > 0.3f) {
			std::cout << "Overlapped with existing wall, skip..." << std::endl;
			return true;
		}
	}

	return false;
}

auto bim::column::serialize() const -> nlohmann::json
{
	std::vector<float> location = { this->pos.x(), this->pos.y(), this->pos.z() };
	std::vector<float> direction = { 0, 0, 1 };

	nlohmann::json obj;
	obj["typeName"] = "GbpWallCircular";
	obj["Radius"] = this->radius;
	obj["SectionRadius"] = this->radius;
	obj["Height"] = this->height;
	obj["LocationPoint"] = location;
	obj["LocationDir"] = direction;
	obj["IsStructural"] = false;

	return obj;
}

