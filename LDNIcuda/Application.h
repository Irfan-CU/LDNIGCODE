#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef APPLICATION_H
#define APPLICATION_H


#include <cstddef> //For size_t.
#include "Slice.h"

	
	class Slice;

	/*!
	 * A singleton class that serves as the starting point for all slicing.
	 *
	 * The application provides a starting point for the slicing engine. It
	 * maintains communication with other applications and uses that to schedule
	 * slices.
	 */
	class Application 
	{
	public:
		/*
		 * \brief The communication currently in use.
		 *
		 * This may be set to ``nullptr`` during the initialisation of the program,
		 * while the correct communication class has not yet been chosen because the
		 * command line arguments have not yet been parsed. In general though you
		 * can assume that it is safe to access this without checking whether it is
		 * initialised.
		

		/*
		 * \brief The slice that is currently ongoing.
		 *
		 * If no slice has started yet, this will be a nullptr.
		 */
		Slice* current_slice;

		static Application& getInstance();

		void run(const size_t argc, GLKObList& meshlist, ContourMesh& c_mesh, SliceDataStorage& storage, int total_layers, std::vector<int>& meshin_layer, double rotBoundingBox[]);	   //const size_t argc is the number pf the meshes need to be sliced

	protected:

		void slice();

	private:
		
		size_t argc;

		/*
		 * \brief An array of C strings containing the arguments that the
		 * application was called with.
		 */
		char** argv;

		Application();

		~Application();
	};

 //Cura namespace.

#endif //APPLICATION_H