#pragma once
#ifndef CLOTH_CUTTING_CONFIG_H
#define CLOTH_CUTTING_CONFIG_H

#include "Common.h"

namespace cloth_cutting {
	
struct Config {
	// ȫ�ֱ���
	// double ת clipper_cInt, �������Ϊ��������С���ű�����clipper �����ĵ��ö���Ҫ�˸ñ���
	static constexpr int scaleRate = 10; 
	// ���߿�ˡ��ɾ����߾��� curveTolerance ���ڵ�����㣬���͹����λ�����������
	static constexpr double curveTolerance = 0.025; 

	Length minGap;     // ���
	Length minPadding; // �߾�

	Config(Length mg = 5, Length mp = 0) : minGap(mg), minPadding(mp) {}
};

}


#endif // !CLOTH_CUTTING_CONFIG_H
