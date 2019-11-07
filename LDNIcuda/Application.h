#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef APPLICATION_H
#define APPLICATION_H


#include <cstddef> //For size_t.

	
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

		void run(const size_t argc, char** argv);

	protected:

		void slice();

	private:
		
		
		Application();

		~Application();
	};

 //Cura namespace.

#endif //APPLICATION_H