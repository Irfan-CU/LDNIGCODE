//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SVG_H
#define SVG_H

#include <stdio.h> // for file output

#include "AABB.h"
#include "IntpointIrfan.h"

	class FPoint3;

	class SVG 
	{
	public:
		enum class Color {
			BLACK,
			WHITE,
			GRAY,
			RED,
			BLUE,
			GREEN,
			YELLOW,
			RAINBOW,
			NONE
		};

	private:

		std::string toString(Color color);

		FILE* out; // the output file
		const AABB aabb; // the boundary box to display
		const curaIrfan::PointIrfan aabb_size;
		const curaIrfan::PointIrfan border;
		const curaIrfan::PointIrfan canvas_size;
		const double scale;
		Color background;

		bool output_is_html;

	public:
		SVG(const char* filename, AABB aabb, curaIrfan::PointIrfan canvas_size = curaIrfan::PointIrfan(1024, 1024), Color background = Color::NONE);

		~SVG();

		/*!
		 * get the scaling factor applied to convert real space to canvas space
		 */
		double getScale() const;

		/*!
		 * transform a curaIrfan::PointIrfan in real space to canvas space
		 */
		curaIrfan::PointIrfan transform(const curaIrfan::PointIrfan& p);

		/*!
		 * transform a curaIrfan::PointIrfan in real space to canvas space with more precision
		 */
		FPoint3 transformF(const curaIrfan::PointIrfan& p);

		void writeComment(std::string comment);

		void writeAreas(const Polygons& polygons, Color color = Color::GRAY, Color outline_color = Color::BLACK, float stroke_width = 1);

		void writeAreas(ConstPolygonRef polygon, Color color = Color::GRAY, Color outline_color = Color::BLACK, float stroke_width = 1);

		void writePoint(const curaIrfan::PointIrfan& p, bool write_coords = false, int size = 5, Color color = Color::BLACK);

		void writePoints(ConstPolygonRef poly, bool write_coords = false, int size = 5, Color color = Color::BLACK);

		void writePoints(Polygons& polygons, bool write_coords = false, int size = 5, Color color = Color::BLACK);

		/*!
		 * \brief Draws a polyline on the canvas.
		 *
		 * The polyline is the set of line segments between each pair of consecutive
		 * curaIrfan::PointIrfans in the specified vector.
		 *
		 * \param polyline A set of curaIrfan::PointIrfans between which line segments must be
		 * drawn.
		 * \param color The colour of the line segments. If this is not specified,
		 * black will be used.
		 */
		void writeLines(std::vector<curaIrfan::PointIrfan> polyline, Color color = Color::BLACK);

		void writeLine(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, Color color = Color::BLACK, float stroke_width = 1);

		void writeLineRGB(const curaIrfan::PointIrfan& from, const curaIrfan::PointIrfan& to, int r = 0, int g = 0, int b = 0, float stroke_width = 1);

		/*!
		 * \brief Draws a dashed line on the canvas from curaIrfan::PointIrfan A to curaIrfan::PointIrfan B.
		 *
		 * This is useful in the case where multiple lines may overlap each other.
		 *
		 * \param a The starting endcuraIrfan::PointIrfan of the line.
		 * \param b The ending endcuraIrfan::PointIrfan of the line.
		 * \param color The stroke colour of the line.
		 */
		void writeDashedLine(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, Color color = Color::BLACK);

		template<typename... Args>
		void printf(const char* txt, Args&&... args);

		void writeText(curaIrfan::PointIrfan p, std::string txt, Color color = Color::BLACK, coord_tIrfan font_size = 10);

		void writePolygons(const Polygons& polys, Color color = Color::BLACK, float stroke_width = 1);

		void writePolygon(ConstPolygonRef poly, Color color = Color::BLACK, float stroke_width = 1);

	};

	template<typename... Args>
	void SVG::printf(const char* txt, Args&&... args)
	{
		fprintf(out, txt, args...);
	}

 // namespace cura
#endif // SVG_H
#pragma once
