#include "Solver.h"
#include <random>
#include <chrono>

namespace cloth_cutting {
	
void Solver::run(){
	List<Piece> origin_pieces;
	preprocess(origin_pieces);
	//List<Piece> clean_pieces;
	//cleanAll(origin_pieces, clean_pieces);
	//offsetAll(clean_pieces, this->pieces);
	List<Piece> offset_pieces;
	offsetAll(origin_pieces, offset_pieces);
	cleanAll(offset_pieces, this->pieces);
	for (auto iter = bins.begin(); iter != bins.end(); ++iter) {
		List<ID> candidate_index = placeCheck(*iter);
		greedyWorker(*iter, candidate_index);
	}
}

void Solver::preprocess(List<Piece>& out_pieces) {
	// plate -> bin
	bins.reserve(input.plates.size());
	for (auto &each_plate : input.plates) {
		auto ox = static_cast<Coord>(each_plate.minPadding);
		auto oy = static_cast<Coord>(each_plate.minPadding);
		auto x = static_cast<Coord>(each_plate.length - each_plate.minPadding);
		auto y = static_cast<Coord>(each_plate.width - each_plate.minPadding);
		bins.emplace_back(point_t(ox, oy), point_t(x, y));
	}

	// item -> piece
	ID piece_index = 0;
	out_pieces.reserve(input.items.size());
	for (int i = 0; i < input.items.size(); ++i) {
		int amount = input.items[i].amount;
		while (amount) {
			Piece piece;
			for (auto &each_xy : input.items[i].raw_coords) {
				bg::append(piece.poly.outer(), point_t(each_xy.x, each_xy.y)); // ��ʱֻ�����⻷
			}
			// ƽ�ƶ���ΰ������½ǵ�����ԭ�㴦������������ֵ
			box_t envelope = bg::return_envelope<box_t>(piece.poly);
			auto offsetX = bg::get<bg::min_corner, 0>(envelope); // auto offsetX2 = envelope.min_corner().x();
			auto offsetY = bg::get<bg::min_corner, 1>(envelope); // auto offsetY2 = envelope.min_corner().y();
			piece.poly.clear();
			for (auto &each_xy : input.items[i].raw_coords) {
				bg::append(piece.poly.outer(), point_t(each_xy.x - offsetX, each_xy.y - offsetY));
			}
			piece.id = piece_index;
			pieceIdMap[piece_index++] = i;
			out_pieces.push_back(piece);
			--amount;
		}
	}
}

/*
* ɾ�����ߡ����ڽӵ������
*/
void Solver::cleanAll(const List<Piece>& in_pieces, List<Piece>& out_pieces) {
	out_pieces.reserve(in_pieces.size());
	for (auto &in_piece : in_pieces) {
		Piece clean_piece;
		in_piece.cleanPiece(clean_piece);
		if (clean_piece.poly.outer().size() > 3 && bg::area(clean_piece.poly)) {
			out_pieces.push_back(clean_piece);
		}
		else {
			std::cout << "origin piece " << in_piece.id << " was deleted." << std::endl;
		}
	}
}

/*
* ���ţ��������ͣ�ȫ�������
*/
void Solver::offsetAll(const List<Piece>& in_pieces, List<Piece>& out_pieces) {
	out_pieces.reserve(in_pieces.size());
	for (auto &in_piece : in_pieces) {
		List<Piece> offset_pieces;
		in_piece.offsetPiece(offset_pieces, 0.5 * config.minGap);
		out_pieces.push_back(offset_pieces[0]);
	}
}

/*
* �������Է����� bin �ϵ� pieces ��ѡ����
*/
List<ID> Solver::placeCheck(const box_t &bin) {
	auto bin_width = bg::get<bg::max_corner, 0>(bin) - bg::get<bg::min_corner, 0>(bin);
	auto bin_height = bg::get<bg::max_corner, 1>(bin) - bg::get<bg::min_corner, 1>(bin);
	List<ID> candidate_index; candidate_index.reserve(pieces.size());
	for (auto iter = pieces.begin(); iter != pieces.end(); ++iter) {
		for (Angle each_angle : input.items[pieceIdMap[(*iter).id]].rotateAngles) {
			polygon_t rotate_poly;
			rotatePolygon((*iter).poly, rotate_poly, each_angle);
			box_t envelope = bg::return_envelope<box_t>(rotate_poly);
			auto width = bg::get<bg::max_corner, 0>(envelope) - bg::get<bg::min_corner, 0>(envelope);
			auto height = bg::get<bg::max_corner, 1>(envelope) - bg::get<bg::min_corner, 1>(envelope);
			if (width < bin_width && height < bin_height) {
				candidate_index.push_back(distance(pieces.begin(), iter));
				break;
			}
		}
	}
	return candidate_index;
}

/*
* �������һ���Ϸ�����ת�Ƕȣ�ͬʱִ����ת����
*/
bool Solver::rotateCheck(const box_t & bin, Piece & piece) {
	auto bin_width = bg::get<bg::max_corner, 0>(bin) - bg::get<bg::min_corner, 0>(bin);
	auto bin_height = bg::get<bg::max_corner, 1>(bin) - bg::get<bg::min_corner, 1>(bin);
	List<Angle> candidate_angles = input.items[pieceIdMap[piece.id]].rotateAngles;
	// candidate_angles �б�ϴ��
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::shuffle(candidate_angles.begin(), candidate_angles.end(), std::default_random_engine(seed));
	for (Angle each_angle : candidate_angles) {
		polygon_t rotate_poly;
		rotatePolygon(piece.poly, rotate_poly, each_angle);
		box_t envelope = bg::return_envelope<box_t>(rotate_poly);
		auto width = bg::get<bg::max_corner, 0>(envelope) - bg::get<bg::min_corner, 0>(envelope);
		auto height = bg::get<bg::max_corner, 1>(envelope) - bg::get<bg::min_corner, 1>(envelope);
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
	for (ID index : candidate_index) {
		candidate_pieces.push_back(pieces[index]);
	}
	sort(candidate_pieces.begin(), candidate_pieces.end(),
		[](Piece &lhs, Piece &rhs) { return bg::area(lhs.poly) < bg::area(rhs.poly); });
	for (auto iter = candidate_pieces.begin(); iter != candidate_pieces.end(); ++iter) {
		if (!rotateCheck(bin, *iter)) { // ��ʱ���������ת
			std::cout << "rotate piece " << (*iter).id << " has no legal angle." << std::endl;
		}
	}
	
	List<NfpPair> nfp_pairs; 
	for (int i = 0; i < candidate_pieces.size(); ++i) {
		// inner nfp
		NfpPair inner_nfp(candidate_pieces[i], bin);
		inner_nfp.nfpPairGenerator();
		nfp_pairs.push_back(inner_nfp);
		// outer nfp
		for (int j = 0; j < i; ++j) {
			NfpPair nfp_pair(candidate_pieces[j], candidate_pieces[i]);
			nfp_pair.nfpPairGenerator();
			nfp_pairs.push_back(nfp_pair);
		}
	}
	HashMap<String, polygon_t> nfp_cache;
	for (auto &each_pair : nfp_pairs) { nfp_cache[each_pair.nfp_key] = each_pair.nfp; }
	placeWorker(bin, candidate_pieces, nfp_cache);
}

/*
* ����ѡ��Ʒ pieces �ŵ� bin ��
*/
Result Solver::placeWorker(const box_t &bin, const List<Piece>& pieces, const HashMap<String, polygon_t>& nfp_cache) {
	List<List<Vector>> all_placed_vectors;
	double fitness = 0;
	double bin_area = bg::area(bin);
	
	while (!pieces.empty()) {
		List<Piece> placed_pieces; // �ѷ��õ����
		List<Vector> placed_vectors; // ��Ӧ vector
		++fitness;
		double min_width = DistanceMax;

		for (auto &curr_piece : pieces) {
#pragma region NfpExistChecker
			// ��� bin_nfp
			String key = getNfpKey(curr_piece);
			if (nfp_cache.find(key) == nfp_cache.end()) {
				std::cout << "inner nfp doesn't exist: " + key << std::endl;
				continue;
			}
			polygon_t bin_nfp = nfp_cache.at(key);
			// ��� curr_piece �������ѷ��õ� each_placed_piece ֮���� pair_nfp
			bool error = false;
			for (auto &each_placed_piece : placed_pieces) {
				key = getNfpKey(each_placed_piece, curr_piece);
				if (nfp_cache.find(key) == nfp_cache.end()) {
					error = true; 
					break;
				}
			}
			if (error) {
				std::cout << "nfp doesn't exist: " + key << std::endl;
				continue; 
			}
#pragma endregion NfpExistChecker

#pragma region FirstPiece
			// ���õ�һ�����
			if (placed_pieces.empty()) {
				Vector position;
				for (auto &nfp_point : bin_nfp.outer()) {
					if (nfp_point.x - curr_piece.poly.outer[0].x < position.x) {
						// Ѱ������ߵ�λ��,�ڷ������ԭʼ����֮��ת����ʸ��
						position = Vector(
							nfp_point.x - curr_piece.poly.outer[0].x,
							nfp_point.y - curr_piece.poly.outer[0].y,
							curr_piece.id,
							curr_piece.rotation
						);
					}
				}
				placed_vectors.push_back(position);
				placed_pieces.push_back(curr_piece);
				continue;
			}
#pragma endregion FirstPiece

#pragma region ClipperExecute
			Paths clipperBinNfp;
			Paths clipperCombinedNfp;
			Paths clipperFinalNfp;
			Clipper clipperUnion;
			Clipper clipperDifference;

			// bin_nfp ת���� clipper paths���� clipperBinNfp.
			clipperBinNfp.push_back(boost2ClipperRing(bin_nfp.outer()));
			for (auto &inner_ring : bin_nfp.inners()) {
				clipperBinNfp.push_back(boost2ClipperRing(inner_ring));
			}

			/* 
			* 1. pair_nfp ת���� clipper paths;
			* 2. clipper paths �󲢼��õ� clipperCombinedNfp.
			*/
			for (int j = 0; j < placed_pieces.size(); ++j) {
				key = getNfpKey(placed_pieces[j], curr_piece);
				polygon_t nfp = nfp_cache.at(key);
				Paths clone;
				clone.push_back(boost2ClipperRing(nfp.outer()));
				for (auto &inner_ring : nfp.inners()) {
					clone.push_back(boost2ClipperRing(inner_ring));
				}
				for (auto &path : clone) {
					for (auto &point : path) {
						point.X += static_cast<cInt>(placed_vectors[j].x * Config::scaleRate);
						point.Y += static_cast<cInt>(placed_vectors[j].y * Config::scaleRate);
					}
				}
				clipperUnion.AddPaths(clone, PolyType::ptSubject, true);
			}
			if (!clipperUnion.Execute(ClipType::ctUnion, clipperCombinedNfp, PolyFillType::pftNonZero, PolyFillType::pftNonZero)) {
				std::cout << "clipperUnion Execute Failed: " + key << std::endl;
				continue;
			}

			/*
			* clipperBinNfp �� clipperCombinedNfp �ü��������õ� clipperFinalNfp.
			*/
			clipperDifference.AddPaths(clipperBinNfp, PolyType::ptSubject, true);
			clipperDifference.AddPaths(clipperCombinedNfp, PolyType::ptClip, true);
			if (!clipperDifference.Execute(ClipType::ctDifference, clipperFinalNfp, PolyFillType::pftNonZero, PolyFillType::pftNonZero))
			{
				std::cout << "clipperDifference Execute Failed: " + key << std::endl;
				continue;
			}

			/*
			* clear clipperFinalNfp
			*/
			CleanPolygons(clipperFinalNfp, 0.0001 * Config::scaleRate);
			clipperFinalNfp.erase(
				std::remove_if(clipperFinalNfp.begin(), clipperFinalNfp.end(),
					[](const Path &path) { return path.size() < 3 || Area(path) < 0.1 * Config::scaleRate * Config::scaleRate; }
				), clipperFinalNfp.end()
			);
			if (clipperFinalNfp.empty()) {
				std::cout << "clipperFinalNfp is empty: " + key << std::endl;
				continue;
			}
#pragma endregion ClipperExecute

		}
	}

	return Result();
}

}
