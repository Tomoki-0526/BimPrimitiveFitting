#pragma once

#include "nlohmann/json.hpp"

#include "Kernel/cylinder.h"

namespace bim {
	class column : public kernel::geom::cylinder
	{
	private:
		// json attributes
		float height;

	public:
		column(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			const Eigen::Vector3f& pos,
			const Eigen::Vector3f& axis,
			float radius,
			float height
		);

		virtual auto overlap(const column& other) -> bool;

		virtual auto serialize() const->nlohmann::json;
	};
}

