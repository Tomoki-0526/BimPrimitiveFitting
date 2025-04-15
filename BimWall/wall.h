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
		wall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& pos, const Eigen::Vector3f& axis, float radius);
		
		bool calc_arc();
		void calc_elev_height();
		bool overlap(const wall& other);

		nlohmann::json serialize() const;
	};
}

