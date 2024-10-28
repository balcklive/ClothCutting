#include "solver.h"
#include <random>
#include <chrono>

namespace cloth_cutting {
	
void Solver::run(){
	List<Piece> origin_pieces, zero_pieces, offset_pieces, clean_pieces;
	preprocess(origin_pieces);
	zeroAll(origin_pieces, zero_pieces);
	offsetAll(zero_pieces, offset_pieces); 
	cleanAll(offset_pieces, this->pieces);
	//cleanAll(zero_pieces, clean_pieces); 
	//offsetAll(clean_pieces, this->pieces);
	auto bin = bins[0];
	List<ID> candidate_index = placeCheck(bin);
	greedyWorker(bin, candidate_index);
}

void Solver::preprocess(List<Piece>& origin_pieces) {
	// plate -> bin
	bins.reserve(input.plates.size());
	for (auto &each_plate : input.plates) {
		auto ox = static_cast<Coord>(each_plate.minPadding);
		auto oy = static_cast<Coord>(each_plate.minPadding);
		auto x = static_cast<Coord>(each_plate.width - each_plate.minPadding);
		auto y = static_cast<Coord>(each_plate.height - each_plate.minPadding);
		bins.emplace_back(point_t(ox, oy), point_t(x, y));
	}

	// item -> piece
	ID piece_index = 0;
	origin_pieces.reserve(input.items.size());
	for (int i = 0; i < input.items.size(); ++i) {
		int amount = input.items[i].amount;
		while (amount) {
			Piece piece;
			for (auto &each_xy : input.items[i].raw_coords) {
				bg::append(piece.poly.outer(), point_t(each_xy.x, each_xy.y)); // ����ֻ���⻷
			}
			piece.id = piece_index;
			pieceIdMap[piece_index++] = i;
			origin_pieces.push_back(piece);
			--amount;
		}
	}

	// todo Defect
}

/*
* �����ж����ƽ�Ƶ�����ԭ�㴦
*/
void Solver::zeroAll(const List<Piece>& in_pieces, List<Piece>& out_pieces) {
	out_pieces.reserve(in_pieces.size());
	for (auto &in_piece : in_pieces) {
		Piece zero_piece;
		in_piece.zeroPiece(zero_piece);
		out_pieces.push_back(zero_piece);
	}
}

/*
* ���ж���μ�
*/
void Solver::cleanAll(const List<Piece>& in_pieces, List<Piece>& out_pieces) {
	out_pieces.reserve(in_pieces.size());
	for (auto &in_piece : in_pieces) {
		Piece clean_piece;
		in_piece.cleanPiece(clean_piece);
		out_pieces.push_back(clean_piece);
	}
}

/*
* ���ж������������ 1/2 Gap
*/
void Solver::offsetAll(const List<Piece>& in_pieces, List<Piece>& out_pieces) {
	out_pieces.reserve(in_pieces.size());
	for (auto &in_piece : in_pieces) {
		Piece offset_piece;
		in_piece.offsetPiece(offset_piece, 0.5 * config.minGap);
		out_pieces.push_back(offset_piece);
	}
}

/*
* �������Է����� bin �ϵ� pieces ��ѡ����
*/
List<ID> Solver::placeCheck(const box_t &bin) {
	auto bin_width = bin.max_corner().x() - bin.min_corner().x();
	auto bin_height = bin.max_corner().y() - bin.min_corner().y();
	List<ID> candidate_index; candidate_index.reserve(pieces.size());
	for (auto & piece : pieces) {
		for (Angle each_angle : input.items[pieceIdMap[piece.id]].rotateAngles) {
			polygon_t rotate_poly;
			rotatePolygon(piece.poly, rotate_poly, each_angle);
			box_t envelope;
			rectangle_t rect = getEnvelope(rotate_poly, envelope);
			auto width = rect.first, height = rect.second;
			if (width < bin_width && height < bin_height) {
				candidate_index.push_back(piece.id);
				break;
			}
		}
	}
	return candidate_index;
}

/*
* ���һ���Ϸ�����ת�Ƕȣ�ִ����ת����
*/
bool Solver::rotateCheck(const box_t & bin, Piece & piece) {
	auto bin_width = bin.max_corner().x() - bin.min_corner().x();
	auto bin_height = bin.max_corner().y() - bin.min_corner().y();
	List<Angle> candidate_angles = input.items[pieceIdMap[piece.id]].rotateAngles;
	// candidate_angles �б�ϴ��
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::shuffle(candidate_angles.begin(), candidate_angles.end(), std::default_random_engine(seed));
	for (Angle each_angle : candidate_angles) {
		polygon_t rotate_poly;
		rotatePolygon(piece.poly, rotate_poly, each_angle);
		box_t envelope;
		rectangle_t rect = getEnvelope(rotate_poly, envelope);
		auto width = rect.first, height = rect.second;
		if (width < bin_width && height < bin_height) {
			piece.rotation = each_angle;
			piece.poly = rotate_poly; // ִ����ת����
			return true;
		}
	}
	return false;
}

/*
* һ��̰�ĵ���
*/
void Solver::greedyWorker(const box_t &bin, const List<ID>& candidate_index) {
	List<Piece> candidate_pieces;
	for (ID index : candidate_index) { candidate_pieces.push_back(pieces[index]); }
	sort(candidate_pieces.begin(), candidate_pieces.end(),
		[](Piece &lhs, Piece &rhs) { return bg::area(lhs.poly) > bg::area(rhs.poly); });
	for (auto &piece : candidate_pieces) {
		if (!rotateCheck(bin, piece)) { // ��ʱ���������ת
			std::cout << "rotate piece " << piece.id << " has no legal angle." << std::endl;
		}
	}
	List<NfpPair> nfp_pairs; 
	for (int i = 0; i < candidate_pieces.size(); ++i) {
		// Inner Fit Polygon
		NfpPair infp_pair(candidate_pieces[i], bin);
		infp_pair.nfpPairGenerator();
		nfp_pairs.push_back(infp_pair);
		// No Fit Polygon
		for (int j = 0; j < i; ++j) {
			NfpPair nfp_pair(candidate_pieces[j], candidate_pieces[i]);
			nfp_pair.nfpPairGenerator();
			nfp_pairs.push_back(nfp_pair);
		}
	}
	HashMap<String, polygon_t> nfp_cache;
	for (auto &each_pair : nfp_pairs) { nfp_cache[each_pair.nfp_key] = each_pair.nfp; }

	List<Piece> placed_pieces;   // �ѷ��õ����
	List<Vector> placed_vectors; // ��Ӧʸ��
	placeWorker(bin, nfp_cache, candidate_pieces, placed_pieces, placed_vectors);

#pragma region ApplyPlacement
	for (auto & each_vector : placed_vectors) {
		this->pieces[each_vector.id].offsetX = each_vector.x;
		this->pieces[each_vector.id].offsetY = each_vector.y;
		this->pieces[each_vector.id].rotation = each_vector.rotation;
	}
#pragma endregion ApplyPlacement
}

/*
* ����ѡ��Ʒ candidate_pieces �ŵ� bin ��
* List<Piece> placed_pieces;   // �ѷ��õ����
* List<Vector> placed_vectors; // ��Ӧʸ��
*/
double Solver::placeWorker(const box_t &bin, const HashMap<String, polygon_t>& nfp_cache, 
						   List<Piece>& candidate_pieces, List<Piece>& placed_pieces, List<Vector>& placed_vectors) {
	double fitness = 0;  // ��Ⱥ�㷨����Ⱥ����Ӧ��
	
	double min_obj_width = DistanceMax;  // Ŀ�꣺��С����ǰ��ʹ�õ����Ͽ��ȣ������Ҳ�����
	
	// ��ʼһ��������ķ���
	for (auto &curr_piece : candidate_pieces) {
		String key;
#pragma region NfpExistChecker
		// ��� Inner Fit Polygon (bin_nfp)
		key = getNfpKey(curr_piece);
		if (nfp_cache.find(key) == nfp_cache.end()) {
			std::cout << "Inner Fit Polygon doesn't exist: " + key << std::endl;
			continue;
		}
		polygon_t bin_nfp = nfp_cache.at(key);
		// ��� curr_piece �������ѷ��õ� placed_pieces ֮���� No Fit Polygon
		bool error = false;
		for (auto &each_placed_piece : placed_pieces) {
			key = getNfpKey(each_placed_piece, curr_piece);
			if (nfp_cache.find(key) == nfp_cache.end()) {
				error = true; 
				break;
			}
		}
		if (error) {
			std::cout << "No Fit Polygon doesn't exist: " + key << std::endl;
			continue; 
		}
#pragma endregion NfpExistChecker
			
		Vector curr_vector;
#pragma region FirstPiece
		// ���õ�һ�����
		if (placed_pieces.empty()) {
			curr_vector.x = DistanceMax;
			auto & refer_point = curr_piece.poly.outer()[0]; // �ο���
			for (auto &nfp_point : bin_nfp.outer()) {
				// Ѱ�� nfp ����ߵ�λ�ã��ڷ������ԭʼ����֮��ת����ʸ��
				if (nfp_point.x() - refer_point.x() < curr_vector.x) {
					curr_vector = Vector(
						curr_piece.id,
						nfp_point.x() - refer_point.x(),
						nfp_point.y() - refer_point.y(),
						curr_piece.rotation
					);
				}
			}
			placed_pieces.push_back(curr_piece);
			placed_vectors.push_back(curr_vector);
			continue;
		}
#pragma endregion FirstPiece

#pragma region ClipperExecute
		Paths clipperUnionNfp;
		Paths clipperFinalNfp;
		Clipper clipperUnion;
		Clipper clipperDifference;

		/*
		* bin_nfp ת���� clipper paths���� clipperBinNfp.
		*/
		Paths clipperBinNfp = boost2ClipperPolygon(bin_nfp);

		/* 
		* nfp ת���� clipper paths, �󲢼��õ� clipperUnionNfp.
		*/
		for (int j = 0; j < placed_pieces.size(); ++j) {
			key = getNfpKey(placed_pieces[j], curr_piece);
			Paths clipperNfp = boost2ClipperPolygon(nfp_cache.at(key));
			for (auto & each_path : clipperNfp) {
				for (auto & each_point : each_path) {
					each_point.X += static_cast<cInt>(placed_vectors[j].x * Config::scaleRate);
					each_point.Y += static_cast<cInt>(placed_vectors[j].y * Config::scaleRate);
				}
			}
			clipperUnion.AddPaths(clipperNfp, PolyType::ptSubject, true);
		}
		if (!clipperUnion.Execute(ClipType::ctUnion, clipperUnionNfp, PolyFillType::pftEvenOdd, PolyFillType::pftEvenOdd)) {
			std::cout << "clipperUnion Execute Failed: " << curr_piece.id << std::endl;
			continue;
		}

		/*
		* clipperBinNfp �� clipperUnionNfp �ü��������õ� clipperFinalNfp.
		*/
		clipperDifference.AddPaths(clipperBinNfp, PolyType::ptSubject, true);
		clipperDifference.AddPaths(clipperUnionNfp, PolyType::ptClip, true);
		if (!clipperDifference.Execute(ClipType::ctDifference, clipperFinalNfp, PolyFillType::pftEvenOdd, PolyFillType::pftEvenOdd))
		{
			std::cout << "clipperDifference Execute Failed: " << curr_piece.id << std::endl;
			continue;
		}

		/*
		* clean clipperFinalNfp
		*/
		CleanPolygons(clipperFinalNfp, 0.0001 * Config::scaleRate);
		clipperFinalNfp.erase(
			std::remove_if(clipperFinalNfp.begin(), clipperFinalNfp.end(), [](const Path &path) { 
				return path.size() < 3 || ClipperLib::Area(path) < 0.1 * Config::scaleRate * Config::scaleRate;
			}), 
			clipperFinalNfp.end()
		);
		if (clipperFinalNfp.empty()) {
			std::cout << "clipperFinalNfp is empty: " << curr_piece.id << std::endl;
			continue;
		}
#pragma endregion ClipperExecute

#pragma region Placement
		double min_eval_width = DistanceMax; // ������ǰ���÷�����ָ�꣺����/���/���Ҳ�����
		double min_shift_X = DistanceMax;
			
		// placed_pieces ����ƫ�ƣ���¼ȫ�־��ΰ���
		double left = bin.max_corner().x(),
				right = bin.min_corner().x(),
				bottom = bin.max_corner().y(),
				top = bin.min_corner().y();
		for (int j = 0; j < placed_pieces.size(); ++j) {
			polygon_t j_poly;
			for (auto & each_point : placed_pieces[j].poly.outer()) {
				bg::append(j_poly.outer(), point_t(each_point.x() + placed_vectors[j].x,
											        each_point.y() + placed_vectors[j].y));
			}
			box_t envelope;
			getEnvelope(j_poly, envelope);
			left = envelope.min_corner().x() < left ? envelope.min_corner().x() : left;
			right = envelope.max_corner().x() > right ? envelope.max_corner().x() : right;
			bottom = envelope.min_corner().y() < bottom ? envelope.min_corner().y() : bottom;
			top = envelope.max_corner().y() > top ? envelope.max_corner().y() : top;
		}

		// �� final_nfp ��ÿ�������Ϸ������
		List<ring_t> final_nfp; 
		final_nfp.reserve(clipperFinalNfp.size());
		for (auto & each_path : clipperFinalNfp) {
			final_nfp.push_back(clipper2BoostRing(each_path));
		}
		for (auto & nfp_ring : final_nfp) {
			auto & refer_point = curr_piece.poly.outer()[0]; // �ο���
			for (auto & nfp_point : nfp_ring) {
				Vector shift_vector(
					curr_piece.id,
					nfp_point.x() - refer_point.x(),
					nfp_point.y() - refer_point.y(),
					curr_piece.rotation
				);

				// curr_piece ����ƫ�ƣ����¾��ΰ���
				polygon_t shift_piece;
				for (auto & each_point : curr_piece.poly.outer()) {
					bg::append(shift_piece.outer(), point_t(each_point.x() + shift_vector.x,
													        each_point.y() + shift_vector.y));
				}
				box_t envelope;
				getEnvelope(shift_piece, envelope);
				left = envelope.min_corner().x() < left ? envelope.min_corner().x() : left;
				right = envelope.max_corner().x() > right ? envelope.max_corner().x() : right;
				bottom = envelope.min_corner().y() < bottom ? envelope.min_corner().y() : bottom;
				top = envelope.max_corner().y() > top ? envelope.max_corner().y() : top;
				auto eval_width = right - left;
				if (!almostEqual(eval_width, min_eval_width) && eval_width < min_eval_width
					|| almostEqual(eval_width, min_eval_width) && shift_vector.x < min_shift_X) {
					// ����ֵ����
					min_eval_width = eval_width;
					min_shift_X = shift_vector.x;
					curr_vector = shift_vector;
					// Ŀ��ֵ����
					min_obj_width = min_eval_width;
				}
			}
		}
		placed_pieces.push_back(curr_piece);
		placed_vectors.push_back(curr_vector);
#pragma endregion Placement
	}

	fitness += min_obj_width / (bin.max_corner().x() - bin.min_corner().x());
	// �����һ��������������ϣ�candidate_pieces �ᱻ���
	for (auto & placed_piece : placed_pieces) {
		candidate_pieces.erase(std::find(candidate_pieces.begin(), candidate_pieces.end(), placed_piece));
	}

	fitness += candidate_pieces.size(); // ���ڿ����ϷŲ��µ���������Ϸ������
	return fitness;
}

void Solver::saveOutput() {
	for (auto & each_piece : pieces) {
		auto &each_item = input.items[pieceIdMap[each_piece.id]];
		each_item.res_coords.clear();
		each_item.res_coords.reserve(each_item.raw_coords.size());
		for (auto &each_xy : each_item.raw_coords) {
			// ������ʱֻ���� 0 �� 180 ��ת
			if (each_piece.rotation == 180) {
				each_item.res_coords.push_back(
					{ each_piece.offsetX - each_xy.x, each_piece.offsetY - each_xy.y }
				);
			}
			else {
				each_item.res_coords.push_back(
					{ each_piece.offsetX + each_xy.x, each_piece.offsetY + each_xy.y }
				);
			}
		}
	}
}

}
