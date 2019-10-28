//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INSET_ORDER_OPTIMIZER_H
#define INSET_ORDER_OPTIMIZER_H

#include "PathOrderOptimizer.h"
#include "SliceDataStorage.h"
#include "PathConfigStorage.h"

	class FffGcodeWriter;
	class LayerPlan;
	class WallOverlapComputation;
	

	class InsetOrderOptimizer
	{
	public:
		/*!
		 * Constructor for inset ordering optimizer
		 * \param gcode_writer The gcode_writer on whose behalf the inset order is being optimized
		 * \param storage where the slice data is stored
		 * \param gcode_layer The initial planning of the gcode of the layer
		 * \param mesh The mesh to be added to the layer plan
		 * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
		 * \param mesh_config the line config with which to print a print feature
		 * \param part The part for which to create gcode
		 * \param layer_nr The current layer number
		 */
		InsetOrderOptimizer(const FffGcodeWriter& gcode_writer, const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr) :
			gcode_writer(gcode_writer),
			storage(storage),
			gcode_layer(gcode_layer),
			extruder_nr(extruder_nr),
			mesh_config(mesh_config),
			part(part),
			layer_nr(layer_nr),
			z_seam_config(EZSeamType::SHARPEST_CORNER, storage.getZSeamHint(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER),
			added_something(false),
			wall_overlapper_0(nullptr),
			wall_overlapper_x(nullptr)
		{
		}
	private:
		
		const FffGcodeWriter& gcode_writer;
		const SliceDataStorage& storage;
		LayerPlan& gcode_layer;
	    const size_t extruder_nr;
		const PathConfigStorage::MeshPathConfigs& mesh_config;
		const SliceLayerPart& part;
		const unsigned int layer_nr;
		const ZSeamConfig z_seam_config;
		bool added_something;
		WallOverlapComputation* wall_overlapper_0;
		WallOverlapComputation* wall_overlapper_x;
		std::vector<std::vector<ConstPolygonPointer>> inset_polys; // vector of vectors holding the inset polygons

		/*!
		 * Generate the insets for the holes of a given layer part after optimizing the ordering.
		 */
		void processHoleInsets();

		/*!
		 * Generate the insets for the outer walls of a given layer part after optimizing the ordering.
		 * \param include_outer true if the outermost inset is to be output
		 * \param include_inners true if the innermost insets are to be output
		 */
		void processOuterWallInsets(const bool include_outer, const bool include_inners);

		/*!
		 * Generate a travel move from the current position to inside the part.
		 * This is used after generating an outer wall so that if a retraction occurs immediately afterwards,
		 * the extruder won't be on the outer wall.
		 */
		void moveInside();

	public:
		/*!
		 * Generate the insets for all of the walls of a given layer part after optimizing the ordering.
		 * \return Whether this function added anything to the layer plan
		 */
		bool processInsetsWithOptimizedOrdering();

		/*!
		 * Test whether it looks to be worthwhile to optimize the inset order of a given layer part.
		 * \param mesh The mesh to be added to the layer plan.
		 * \param part The part for which to create gcode
		 * \return true if it is worth optimizing the inset order, false if not
		 */
		static bool optimizingInsetsIsWorthwhile(const SliceLayerPart& part);
	};

 //namespace cura

#endif // INSET_ORDER_OPTIMIZER_H
