#include "piece.h"
#include <cassert>

namespace cloth_cutting {

/*
* ɾ�����ߣ��򼸺����ߣ��Ͱ��ڽӵĶ���
* ��ʱֻ�����⻷
*/
void Piece::cleanPiece(Piece &res_piece) const {
	Path in_poly = boost2ClipperRing(poly.outer());
	Paths out_polys;
	// �򻯶����
	SimplifyPolygon(in_poly, out_polys, PolyFillType::pftNonZero);
	if (out_polys.empty()) { return; }
	
	Path biggest_poly = out_polys[0];
	double biggest_area = ClipperLib::Area(biggest_poly);
	for (auto &each_poly : out_polys) {
		double each_area = ClipperLib::Area(each_poly);
		biggest_poly = each_area > biggest_area ? each_poly : biggest_poly;
		biggest_area = each_area > biggest_area ? each_area : biggest_area;
	}
	// ɾ������
	CleanPolygon(biggest_poly, Config::curveTolerance * Config::scaleRate);
	if (biggest_poly.empty()) { return; }

	res_piece.id = this->id;
	res_piece.rotation = this->rotation;
	res_piece.offsetX = this->offsetX;
	res_piece.offsetY = this->offsetY;
	res_piece.poly.outer() = clipper2BoostRing(biggest_poly);
}

/*
* ���Ŷ����
* offset < 0��һ������ο��ܻᱻ��ֳɶ��С�����
* offset > 0���������ͣ�ֻ��һ������Σ���ʱֻ������һ�����
*/
void Piece::offsetPiece(List<Piece>& res_pieces, double offset) const {
	if (offset == 0) { return; }
	Path in_poly = boost2ClipperRing(poly.outer());
	Paths out_polys;
	// JoinType = jtMiter��MiterLimit���ã�JoinType = jtRound��ArcTolerance���á�
	// JoinType = jtRound��ʹ�û��߰������
	ClipperOffset co(2.0, Config::curveTolerance * Config::scaleRate);
	co.AddPath(in_poly, JoinType::jtRound, EndType::etClosedPolygon);
	co.Execute(out_polys, offset * Config::scaleRate);

	res_pieces.resize(out_polys.size());
	for (int i = 0; i < out_polys.size(); ++i) {
		res_pieces[i].id = this->id;
		res_pieces[i].rotation = this->rotation;
		res_pieces[i].offsetX = this->offsetX;
		res_pieces[i].offsetY = this->offsetY;
		res_pieces[i].poly.outer() = clipper2BoostRing(out_polys[i]);
	}
	if (offset > 0) { assert(res_pieces.size() == 1); }
}

}