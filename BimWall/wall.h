#pragma once

#include "nlohmann/json.hpp"

#include "Kernel/cylinder.h"

namespace bim {
	class wall : public kernel::geom::cylinder
	{
	private:
		float zmin;
		float zmax;

		// json attributes
		float height;
		std::vector<float> start_point;
		std::vector<float> end_point;
		std::vector<float> mid_point;

	public:
		wall(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			const Eigen::Vector3f& pos,
			const Eigen::Vector3f& axis,
			float radius
		);
		
		auto calc_arc() -> bool;
		auto calc_elev_height() -> void;
		virtual auto overlap(const wall& other) -> bool;

		virtual auto serialize() const -> nlohmann::json;
	};
}

