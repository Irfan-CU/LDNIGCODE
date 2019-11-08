#pragma once
//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_GROUP_H
#define MESH_GROUP_H

#include "mesh.h"
#include "NoCopy.h"


	class FMatrix3x3;

	class MeshGroup : public NoCopy
	{
	public:
		std::vector<Mesh> meshes;
		Settings settings;

		Point3 min_mesh() const; //! minimal corner of bounding box
		Point3 max_mesh() const; //! maximal corner of bounding box

		void clear();

		void finalize();
	};

	

#endif //MESH_GROUP_H
