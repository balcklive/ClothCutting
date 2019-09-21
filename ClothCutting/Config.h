#pragma once
#ifndef CLOTH_CUTTING_CONFIG_H
#define CLOTH_CUTTING_CONFIG_H

#include "Common.h"

namespace cloth_cutting {
	
struct Config {
	// ȫ�ֱ���
	// double ת clipper_cInt, �������Ϊ��������С���ű���������clipper�������ã�ע��˱���
	static constexpr int scaleRate = 10; 
	// ���߿�ˡ��ɾ����߾��� curveTolerance ���ڵ�����㣬���͹����λ�����������
	static constexpr double curveTolerance = 0.02; 

	Length minGap;       // ���
	Length minPadding;   // �߾�
	int populationSize;  // ��Ⱥ����
	double mutationRate; // ͻ����

	Config(Length mg = 5, Length mp = 0, int ps = 10, double mr = 0.1) :
		minGap(mg), minPadding(mp), populationSize(ps), mutationRate(mr) {}
};

}


#endif // !CLOTH_CUTTING_CONFIG_H
