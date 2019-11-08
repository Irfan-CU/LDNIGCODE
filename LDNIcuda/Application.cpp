//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifdef _OPENMP
#include <omp.h> // omp_get_num_threads
#endif // _OPENMP
#include <string>
#include <vector>
#include "Application.h"


//To use the command line to slice stuff.
#include "FffProcessor.h"
#include "Progress.h"
#include "logoutput.h"
#include "String.h" //For stringcasecompare.

	Application::Application()
		: current_slice(0)
	{
	}

	Application::~Application()
	{
		
	}

	Application& Application::getInstance()
	{
		static Application instance; //Constructs using the default constructor.
		return instance;
	}

	
	void Application::run(const size_t argc, GLKObList& meshlist, ContourMesh& c_mesh, SliceDataStorage& storage, int total_layers, std::vector<int>& meshin_layer, double rotBoundingBox[])
	{
		if (argc <= 0)
		{
			return; //Don't slice empty mesh groups.
		}

		Slice slice(argc);
		printf("the scene meh group size is %d \n", slice.scene.mesh_groups.size());
		const size_t extruder_count = slice.scene.settings.get<size_t>("machine_extruder_count");
		printf("the extruder count is %d \n", extruder_count);
		for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
		{
			slice.scene.extruders.emplace_back(extruder_nr);
		}
		slice.compute(meshlist, c_mesh,  meshin_layer, total_layers, rotBoundingBox);
		//FffProcessor::getInstance()->finalize();
		slice.reset();
	}

 //Cura namespace.