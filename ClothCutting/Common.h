////////////////////////////////
/// usage : 1.	common type aliases.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <set>
#include <map>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <limits>

namespace cloth_cutting {

// zero-based consecutive integer identifier.
using ID = int;
// �������ͣ����� boost_geometry, Clipper ��������Ϊ cInt��
using Coord = double;
const auto DistanceMax = (std::numeric_limits<Coord>::max)();
// ��Ŀ���������ϳ���覴ð뾶����������Ϊ����
using Length = int;
// ��ת�Ƕ� [0, 90, 180, 270, 360]
using Angle = int;

template<typename T>
using List = std::vector<T>;

template<typename Key>
using HashSet = std::unordered_set<Key>;

template<typename T>
using Set = std::set<T>;

template<typename  Key, typename Val>
using HashMap = std::unordered_map<Key, Val>;

template<typename Key, typename Val>
using Map = std::map<Key, Val>;

using String = std::string;

}


#endif // COMMON_H
