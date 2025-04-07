#pragma once

#include <vector>

namespace kernel {
	namespace alg {
		/*
		 x: ���з�ֵ���ź�����
		 plateau_size: ��֤���Ӧ��ƽ����Ŀ���ڸ���ֵ
		 height: ����ָ��height���źŶ�������
		 threshold: �������������Ĵ�ֱ����
		 distance: ���ڷ�֮�����Сˮƽ����, ���Ƴ���С�ķ壬ֱ������ʣ��������������Ϊֹ
		 prominence: ͻ��̶�
		 wlen:
		 width: ����Ŀ��
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