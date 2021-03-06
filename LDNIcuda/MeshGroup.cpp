//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include<limits>

#include "MeshGroup.h"
#include "floatpoint.h"

#include "logoutput.h"
#include "String.h"


	FILE* binaryMeshBlob = nullptr;

	/* Custom fgets function to support Mac line-ends in Ascii STL files. OpenSCAD produces this when used on Mac */
	void* fgets_(char* ptr, size_t len, FILE* f)
	{
		while (len && fread(ptr, 1, 1, f) > 0)
		{
			if (*ptr == '\n' || *ptr == '\r')
			{
				*ptr = '\0';
				return ptr;
			}
			ptr++;
			len--;
		}
		return nullptr;
	}

    Point3 MeshGroup::min_mesh() const
	{
		if (meshes.size() < 1)
		{
			return Point3(0, 0, 0);
		}
		Point3 ret(std::numeric_limits<coord_tIrfan>::max(), std::numeric_limits<coord_tIrfan>::max(), std::numeric_limits<coord_tIrfan>::max());
		for (const Mesh& mesh : meshes)
		{
			if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("anti_overhang_mesh")) //Don't count pieces that are not printed.
			{
				continue;
			}
			Point3 v = mesh.min();
			ret.x = std::min(ret.x, v.x);
			ret.y = std::min(ret.y, v.y);
			ret.z = std::min(ret.z, v.z);
		}
		return ret;
	}

	Point3 MeshGroup::max_mesh() const
	{
		if (meshes.size() < 1)
		{
			return Point3(0, 0, 0);
		}
		Point3 ret(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min());
		for (const Mesh& mesh : meshes)
		{
			if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("anti_overhang_mesh")) //Don't count pieces that are not printed.
			{
				continue;
			}
			Point3 v = mesh.max();
			ret.x = std::max(ret.x, v.x);
			ret.y = std::max(ret.y, v.y);
			ret.z = std::max(ret.z, v.z);
		}
		return ret;
	}

	void MeshGroup::clear()
	{
		for (Mesh& m : meshes)
		{
			m.clear();
		}
	}

	void MeshGroup::finalize()
	{
		//If the machine settings have been supplied, offset the given position vertices to the center of vertices (0,0,0) is at the bed center.
		Point3 meshgroup_offset(0, 0, 0);
		if (!settings.get<bool>("machine_center_is_zero"))
		{
			meshgroup_offset.x = settings.get<coord_tIrfan>("machine_width") / 2;
			meshgroup_offset.y = settings.get<coord_tIrfan>("machine_depth") / 2;
		}

		// If a mesh position was given, put the mesh at this position in 3D space. 
		for (Mesh& mesh : meshes)
		{
			Point3 mesh_offset(mesh.settings.get<coord_tIrfan>("mesh_position_x"), mesh.settings.get<coord_tIrfan>("mesh_position_y"), mesh.settings.get<coord_tIrfan>("mesh_position_z"));
			if (mesh.settings.get<bool>("center_object"))
			{
				Point3 object_min = mesh.min();
				Point3 object_max = mesh.max();
				Point3 object_size = object_max - object_min;
				mesh_offset += Point3(-object_min.x - object_size.x / 2, -object_min.y - object_size.y / 2, -object_min.z);
			}
			//mesh.offset(mesh_offset + meshgroup_offset);
		}
	}

	bool loadMeshSTL_ascii(Mesh* mesh, const char* filename, const FMatrix3x3& matrix)
	{
		FILE* f = fopen(filename, "rt");
		char buffer[1024];
		FPoint3 vertex;
		int n = 0;
		Point3 v0(0, 0, 0), v1(0, 0, 0), v2(0, 0, 0);
		while (fgets_(buffer, sizeof(buffer), f))
		{
			if (sscanf(buffer, " vertex %f %f %f", &vertex.x, &vertex.y, &vertex.z) == 3)
			{
				n++;
				switch (n)
				{
				case 1:
					v0 = matrix.apply(vertex);
					break;
				case 2:
					v1 = matrix.apply(vertex);
					break;
				case 3:
					v2 = matrix.apply(vertex);
					//mesh->addFace(v0, v1, v2);
					n = 0;
					break;
				}
			}
		}
		fclose(f);
		//mesh->finish();
		return true;
	}

	

//namespace cura
