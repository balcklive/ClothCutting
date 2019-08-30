#pragma once
#ifndef CLOTH_CUTTING_BOOST_UTILS_HPP
#define CLOTH_CUTTING_BOOST_UTILS_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "../Common.h"

namespace cloth_cutting {

namespace bg = boost::geometry;

using T = Coord;

// n ά������
template<size_t dimension = 2>
using point_base = bg::model::point<T, dimension, bg::cs::cartesian>;

/*****************************
* ���¶���ȫ����Զ�ά������ *
******************************/

// ��ά�����
using point_t = bg::model::d2::point_xy<T>;
const point_t originPoint(0, 0);

// ����
using linestring_t = bg::model::linestring<point_t>;

// ����Σ���ʱ�룬���=�յ㣬����0�����߶���ڻ� inner rings��
using polygon_t = bg::model::polygon<point_t, false, true>;

// �㼯��
using multi_point_t = bg::model::multi_point<point_t>;

// ���߼���
using multi_linestring_t = bg::model::multi_linestring<linestring_t>;

// ����μ���
using multi_polygon_t = bg::model::multi_polygon<polygon_t>;

// ����
using box_t = bg::model::box<point_t>;

// ������ʱ�룬���=�յ㣩
using ring_t = bg::model::ring<point_t, false, true>;

// �߶�
using segment_t = bg::model::segment<point_t>;

}

#endif // !CLOTH_CUTTING_BOOST_UTILS_HPP
