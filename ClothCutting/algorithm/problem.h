#pragma once
#ifndef CLOTH_CUTTING_PROBLEM_H
#define CLOTH_CUTTING_PROBLEM_H

#include <fstream>
#include <sstream>
#include <iostream>
#include "../Common.h"

namespace cloth_cutting {

struct Coordinate {
public:
	Coord x, y;
};

// ���
class Item {
public:
	String batchIndex; // ���κ�
	String itemIndex; // ����� 
	int amount; // �������
	List<Coordinate> raw_coords; // �����ԭʼ��������
	List<Coordinate> res_coords; // ������������������
	List<int> rotateAngles; // ����ת�ĽǶ�
	String plateIndex; // ���Ϻ�
};

// 覴�
class Defect {
public:
    Coordinate center; // Բ��
    Length radius; // �뾶
};

// ����
class Plate {
public:
	String plateIndex; // ���Ϻ�
	Length length, width; // ���Ϳ�
	List<Defect> defects; // ���ϵ�覴�
	Length minGap; // ��������С���
	Length minPadding; // ��С�߾�
};

// ����
class Input {
public:
	List<Item> items; // ����б�
	List<Plate> plates; // �����б�
    
    Input();
    void readItems(std::ifstream &fin);
    void readPlates(std::ifstream &fin);
    String Trim(String& str);
};

// ���
class Output {
public:
    void save(List<Item> &items);
};

}

#endif // !CLOTH_CUTTING_PROBLEM_H
