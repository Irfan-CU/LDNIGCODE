#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_H
#define MESH_H

#include <unordered_map>

#include "AABB3D.h"

class Mesh
{
		//! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
		std::unordered_map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
		AABB3D aabb;
	public:
		
		std::string mesh_name="mesh1";
		//Mesh();
		void clear();
		Point3 min() const; //!< min (in x,y and z) vertex of the bounding box
		Point3 max() const; //!< max (in x,y and z) vertex of the bounding box
		AABB3D getAABB() const; //!< Get the axis aligned bounding box
		
		/*!
		 * Offset the whole mesh (all vertices and the bounding box).
		 * \param offset The offset byu which to offset the whole mesh.
		 */
	private:
		
};

//namespace cura
#endif//MESH_H


