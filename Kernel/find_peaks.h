#pragma once

#include <vector>

namespace kernel {
	namespace alg {
		/*
		 x: 带有峰值的信号序列
		 plateau_size: 保证峰对应的平顶数目大于给定值
		 height: 低于指定height的信号都不考虑
		 threshold: 其与相邻样本的垂直距离
		 distance: 相邻峰之间的最小水平距离, 先移除较小的峰，直到所有剩余峰的条件都满足为止
		 prominence: 突起程度
		 wlen:
		 width: 波峰的宽度
		 rel_height:
		 */
		auto find_peaks(
			std::vector<double> x, 
			std::vector<double> plateau_size = {}, 
			std::vector<double> height = {}, 
			std::vector<double> threshold = {}, 
			int distance = 0, 
			std::vector<double> prominence = {}, 
			int wlen = 2, 
			std::vector<double> width = {}, 
			double rel_height = 0.5
		) -> std::vector<int>;
	}
}