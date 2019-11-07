//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifdef _OPENMP
#include <omp.h> // omp_get_num_threads
#endif // _OPENMP
#include <string>
#include <vector>
#include "Application.h"

//To use the command line to slice stuff.
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

	void Application::slice()
	{
		
	}

	void Application::run(const size_t argc, char** argv)
	{
		slice();
	
	}

 //Cura namespace.