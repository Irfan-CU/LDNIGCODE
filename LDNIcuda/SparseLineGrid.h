#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_SPARSE_LINE_GRID_H
#define UTILS_SPARSE_LINE_GRID_H

#include <cassert>
#include <unordered_map>
#include <vector>
#include <functional>

#include "IntpointIrfan.h"
#include "SparseGrid.h"
#include "SVG.h" // debug

/*! \brief Sparse grid which can locate spatially nearby elements efficiently.
	 *
	 * \tparam ElemT The element type to store.
	 * \tparam Locator The functor to get the start and end locations from ElemT.
	 *    must have: std::pair<Point, Point> operator()(const ElemT &elem) const
	 *    which returns the location associated with val.
	 */
	template<class ElemT, class Locator>
	class SparseLineGrid : public SparseGrid<ElemT>
	{
	public:
		using Elem = ElemT;

		/*! \brief Constructs a sparse grid with the specified cell size.
		 *
		 * \param[in] cell_size The size to use for a cell (square) in the grid.
		 *    Typical values would be around 0.5-2x of expected query radius.
		 * \param[in] elem_reserve Number of elements to research space for.
		 * \param[in] max_load_factor Maximum average load factor before rehashing.
		 */
		SparseLineGrid(coord_tIrfan cell_size, size_t elem_reserve = 0U, float max_load_factor = 1.0f);

		/*! \brief Inserts elem into the sparse grid.
		 *
		 * \param[in] elem The element to be inserted.
		 */
		void insert(const Elem &elem);

		void debugHTML(std::string filename);

		static void debugTest();
	protected:
		using GridPoint = typename SparseGrid<ElemT>::GridPoint;
		using grid_coord_t = typename SparseGrid<ElemT>::grid_coord_t;

		/*! \brief Accessor for getting locations from elements. */
		Locator m_locator;
	};



#define SGI_TEMPLATE template<class ElemT, class Locator>
#define SGI_THIS SparseLineGrid<ElemT, Locator>

	SGI_TEMPLATE
		SGI_THIS::SparseLineGrid(coord_tIrfan cell_size, size_t elem_reserve, float max_load_factor)
		: SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor)
	{
	}

	SGI_TEMPLATE
		void SGI_THIS::insert(const Elem &elem)
	{
		const std::pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan> line = m_locator(elem);
		using GridMap = std::unordered_multimap<GridPoint, Elem>;
		// below is a workaround for the fact that lambda functions cannot access private or protected members
		// first we define a lambda which works on any GridMap and then we bind it to the actual protected GridMap of the parent class
		std::function<bool(GridMap*, const GridPoint)> process_cell_func_ = [&elem, this](GridMap* m_grid, const GridPoint grid_loc)
		{
			m_grid->emplace(grid_loc, elem);
			return true;
		};
		using namespace std::placeholders;  // for _1, _2, _3...
		GridMap* m_grid = &(this->m_grid);
		std::function<bool(const GridPoint)> process_cell_func(std::bind(process_cell_func_, m_grid, _1));

		SparseGrid<ElemT>::processLineCells(line, process_cell_func);
	}

	SGI_TEMPLATE
		void SGI_THIS::debugHTML(std::string filename)
	{
		AABB aabb;
		for (std::pair<GridPoint, ElemT> cell : SparseGrid<ElemT>::m_grid)
		{
			aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first));
			aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), SparseGrid<ElemT>::nonzero_sign(cell.first.Y))));
		}
		SVG svg(filename.c_str(), aabb);
		for (std::pair<GridPoint, ElemT> cell : SparseGrid<ElemT>::m_grid)
		{
			// doesn't draw cells at x = 0 or y = 0 correctly (should be double size)
			curaIrfan::PointIrfan, lb = SparseGrid<ElemT>::toLowerCorner(cell.first);
			curaIrfan::PointIrfan, lt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(0, SparseGrid<ElemT>::nonzero_sign(cell.first.Y)));
			curaIrfan::PointIrfan, rt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), SparseGrid<ElemT>::nonzero_sign(cell.first.Y)));
			curaIrfan::PointIrfan, rb = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), 0));
			if (lb.X == 0)
			{
				lb.X = -SparseGrid<ElemT>::m_cell_size;
				lt.X = -SparseGrid<ElemT>::m_cell_size;
			}
			if (lb.Y == 0)
			{
				lb.Y = -SparseGrid<ElemT>::m_cell_size;
				rb.Y = -SparseGrid<ElemT>::m_cell_size;
			}
			//         svg.writePoint(lb, true, 1);
			svg.writeLine(lb, lt, SVG::Color::GRAY);
			svg.writeLine(lt, rt, SVG::Color::GRAY);
			svg.writeLine(rt, rb, SVG::Color::GRAY);
			svg.writeLine(rb, lb, SVG::Color::GRAY);

			std::pair<curaIrfan::PointIrfan, , curaIrfan::PointIrfan, > line = m_locator(cell.second);
			svg.writePoint(line.first, true);
			svg.writePoint(line.second, true);
			svg.writeLine(line.first, line.second, SVG::Color::BLACK);
		}
	}

	SGI_TEMPLATE
		void SGI_THIS::debugTest()
	{
		struct PairLocator
		{
			std::pair<curaIrfan::PointIrfan, , curaIrfan::PointIrfan, > operator()(const std::pair<curaIrfan::PointIrfan, , curaIrfan::PointIrfan, >& val) const
			{
				return val;
			}
		};
		SparseLineGrid<std::pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>, PairLocator> line_grid(10);

		// straight lines
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(50, 0), curaIrfan::PointIrfan(50, 70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(0, 90), curaIrfan::PointIrfan(50, 90)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(253, 103), curaIrfan::PointIrfan(253, 173)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(203, 193), curaIrfan::PointIrfan(253, 193)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-50, 0), curaIrfan::PointIrfan(-50, -70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(0, -90), curaIrfan::PointIrfan(-50, -90)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-253, -103), curaIrfan::PointIrfan(-253, -173)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-203, -193), curaIrfan::PointIrfan(-253, -193)));

		// diagonal lines
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(113, 133), curaIrfan::PointIrfan(166, 125)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(13, 73), curaIrfan::PointIrfan(26, 25)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(166, 33), curaIrfan::PointIrfan(113, 25)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(26, 173), curaIrfan::PointIrfan(13, 125)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-24, -18), curaIrfan::PointIrfan(-19, -64)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-113, -133), curaIrfan::PointIrfan(-166, -125)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-166, -33), curaIrfan::PointIrfan(-113, -25)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-26, -173), curaIrfan::PointIrfan(-13, -125)));

		// diagonal lines exactly crossing cell corners
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(160, 190), curaIrfan::PointIrfan(220, 170)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(60, 130), curaIrfan::PointIrfan(80, 70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(220, 90), curaIrfan::PointIrfan(160, 70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(80, 220), curaIrfan::PointIrfan(60, 160)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-160, -190), curaIrfan::PointIrfan(-220, -170)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-60, -130), curaIrfan::PointIrfan(-80, -70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-220, -90), curaIrfan::PointIrfan(-160, -70)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-80, -220), curaIrfan::PointIrfan(-60, -160)));

		// single cell
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(203, 213), curaIrfan::PointIrfan(203, 213)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(223, 213), curaIrfan::PointIrfan(223, 215)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(243, 213), curaIrfan::PointIrfan(245, 213)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(263, 213), curaIrfan::PointIrfan(265, 215)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(283, 215), curaIrfan::PointIrfan(285, 213)));
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(-203, -213), curaIrfan::PointIrfan(-203, -213)));

		// around origin
		line_grid.insert(std::make_pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan>(curaIrfan::PointIrfan(20, -20), curaIrfan::PointIrfan(-20, 20)));

		line_grid.debugHTML("line_grid.html");
	}


#undef SGI_TEMPLATE
#undef SGI_THIS

 // namespace cura

#endif // UTILS_SPARSE_LINE_GRID_H
