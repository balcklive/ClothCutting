#include "piece.h"
#include <cassert>

namespace cloth_cutting {
	/*
	* �������ƽ�Ƶ�����ԭ��
	*/
	void Piece::zeroPiece(Piece & res_piece) const {
		box_t envelope;
		getEnvelope(this->poly, envelope);
		auto moveX = 0 - envelope.min_corner().x();
		auto moveY = 0 - envelope.min_corner().y();
		translatePolygon(this->poly, res_piece.poly, moveX, moveY);
		res_piece.id = this->id;
		res_piece.rotation = this->rotation;
		res_piece.offsetX = this->offsetX;
		res_piece.offsetY = this->offsetY;
	}

	/*
	* �򻯶���Σ�ɾ�����ཻ�����ߣ��򼸺����ߣ��Ͱ��ڽӵĶ���
	*/
	void Piece::cleanPiece(Piece &res_piece) const {
		Paths polys = boost2ClipperPolygon(this->poly);

		// ɾ�����ཻ
		SimplifyPolygons(polys, PolyFillType::pftEvenOdd);

		// ɾ�����߶���
		CleanPolygons(polys, Config::curveTolerance * Config::scaleRate);
		
		assert(polys.size() == 1);

		res_piece.id = this->id;
		res_piece.rotation = this->rotation;
		res_piece.offsetX = this->offsetX;
		res_piece.offsetY = this->offsetY;
		res_piece.poly = clipper2BoostPolygon(polys);
	}

	/*
	* ���Ŷ����
	* offset > 0����������ȷ��ֻ�����һ�������
	* offset < 0��һ������ο��ܻᱻ��ֳɶ��С����Σ���֧��
	*/
	void Piece::offsetPiece(Piece &res_piece, double offset) const {
		if (offset <= 0) { 
			res_piece.poly = this->poly;
		}
		else {
			Paths in_polys = boost2ClipperPolygon(this->poly);
			Paths out_polys;
			// JoinType = jtMiter�����������ضϼ�ǣ�MiterLimit = 2.0
			// JoinType = jtRound��ʹ�û��߰������, ArcTolerance = Config::curveTolerance * Config::scaleRate
			ClipperOffset co(2.0, Config::curveTolerance * Config::scaleRate);
			co.AddPaths(in_polys, JoinType::jtRound, EndType::etClosedPolygon);
			co.Execute(out_polys, offset * Config::scaleRate);
			assert(out_polys.size() == 1);
			res_piece.poly = clipper2BoostPolygon(out_polys);
		}
		res_piece.id = this->id;
		res_piece.rotation = this->rotation;
		res_piece.offsetX = this->offsetX;
		res_piece.offsetY = this->offsetY;
	}

}