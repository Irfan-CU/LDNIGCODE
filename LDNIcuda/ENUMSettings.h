#pragma once
#ifndef SETTINGSBASEVIRTUAL_H
#define SETTINGSBASEVIRTUAL_H

enum class EFillMethod
{
	LINES,
	GRID,
	CUBIC,
	CUBICSUBDIV,
	TETRAHEDRAL,
	QUARTER_CUBIC,
	TRIANGLES,
	TRIHEXAGON,
	CONCENTRIC,
	ZIG_ZAG,
	CROSS,
	CROSS_3D,
	GYROID,
	NONE
};

enum class EGCodeFlavor

{
	MARLIN = 0,
	GRIFFIN = 6,

};
enum class EZSeamType
{
	RANDOM,
	SHORTEST,
	USER_SPECIFIED,
	SHARPEST_CORNER
};
enum class EZSeamCornerPrefType
{
	Z_SEAM_CORNER_PREF_NONE,
	Z_SEAM_CORNER_PREF_INNER,
	Z_SEAM_CORNER_PREF_OUTER,
	Z_SEAM_CORNER_PREF_ANY,
	Z_SEAM_CORNER_PREF_WEIGHTED
};


enum class CombingMode
{
	OFF,
	ALL,
	NO_SKIN,
	INFILL
};

enum class SlicingTolerance
{
	MIDDLE,
	INCLUSIVE,
	EXCLUSIVE
};

enum class ESurfaceMode
{
	NORMAL,
	SURFACE,
	BOTH
};

enum class EPlatformAdhesion
{
	SKIRT,
	BRIM,
	RAFT,
	NONE
};

enum class FillPerimeterGapMode
{
	NOWHERE,
	EVERYWHERE
};

#endif