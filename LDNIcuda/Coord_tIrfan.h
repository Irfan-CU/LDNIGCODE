#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_COORD_IrfanH
#define UTILS_COORD_IrfanH


//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <clipper.hpp>


using coord_tIrfan = ClipperLib::cInt;

#define INT2MM(n) (double(n) / 1000.0)
#define INT2MM2(n) (double(n) / 1000000.0)
#define MM2INT(n) (coord_tIrfan((n) * 1000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM2_2INT(n) (coord_tIrfan((n) * 1000000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM3_2INT(n) (coord_tIrfan((n) * 1000000000 + 0.5 * (((n) > 0) - ((n) < 0))))

#define INT2MICRON(n) ((n) / 1)
#define MICRON2INT(n) ((n) * 1)
 // namespace cura


#endif // UTILS_COORD_T_H
