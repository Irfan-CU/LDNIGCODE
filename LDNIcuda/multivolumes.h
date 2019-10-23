//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MULTIVOLUMES_H
#define MULTIVOLUMES_H

#include <vector>

/* This file contains code to help fixing up and changing layers that are built from multiple volumes. */

class Slicer;



void carveMultipleVolumes(std::vector<Slicer*> &volumes);

void generateMultipleVolumesOverlap(std::vector<Slicer*> &meshes);



//namespace cura

#endif//MULTIVOLUMES_H
