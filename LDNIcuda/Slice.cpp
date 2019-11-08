//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <vector>

#include "ExtruderTrain.h"
#include "Slice.h"
#include "logoutput.h"

Slice::Slice(const size_t num_mesh_groups)
		: scene(num_mesh_groups)
	{}

	void Slice::compute(GLKObList& meshlist, ContourMesh& c_mesh, std::vector<int>& meshin_layer, int total_layers, double rotBoundingBox[])
	{
		//printf("Warning %s", scene.getAllSettingsString().c_str());
		
		for (std::vector<MeshGroup>::iterator mesh_group = scene.mesh_groups.begin(); mesh_group != scene.mesh_groups.end(); mesh_group++)
		{
			scene.current_mesh_group = mesh_group;
			for (ExtruderTrain& extruder : scene.extruders)
			{
				extruder.settings.setParent(&scene.current_mesh_group->settings);
			}
			
			scene.processMeshGroup(*mesh_group,meshlist,c_mesh,  meshin_layer, total_layers, rotBoundingBox);
		}



	}

	void Slice::reset()
	{
		scene.extruders.clear();
		scene.mesh_groups.clear();
		scene.settings = Settings();
	}

