#pragma once

#ifndef GCODE_WRITER_H
#define GCODE_WRITER_H

#include <fstream>
#include "FanSpeedLayerTime.h"
#include "GcodeExport.h"
#include "LayerBufferPlan.h"
#include "PathConfigStorage.h"


namespace std
{
template<typename T> class optional;
}

class Polygons;
class SliceDataStorage;
class SliceMeshStorage;
class SliceLayer;
class SliceLayerPart;


class FffGcodeWriter
{
public:
	coord_tIrfan max_object_height; //!< The maximal height of all previously sliced meshgroups, used to avoid collision when moving to the next meshgroup to print.

 /*
  * Buffer for all layer plans (of type LayerPlan)
  *
  * The layer plans are buffered so that we can start heating up a nozzle several layers before it needs to be used.
  * Another reason is to perform Auto Temperature.
  */
	LayerPlanBuffer layer_plan_buffer;

	/*!
	 * The class holding the current state of the gcode being written.
	 *
	 * It holds information such as the last written position etc.
	 */
	GCodeExport gcode;

	std::ofstream output_file;

	/*!
	 * For each raft/filler layer, the extruders to be used in that layer in the order in which they are going to be used.
	 * The first number is the first raft layer. Indexing is shifted compared to normal negative layer numbers for raft/filler layers.
	 */
	std::vector<std::vector<size_t>> extruder_order_per_layer_negative_layers;

	std::vector<std::vector<size_t>> extruder_order_per_layer; //!< For each layer, the extruders to be used in that layer in the order in which they are going to be used

	std::vector<std::vector<size_t>> mesh_order_per_extruder; //!< For each extruder, the order of the meshes (first element is first mesh to be printed)

	/*!
	 * For each extruder on which layer the prime will be planned,
	 * or a large negative number if it's already planned outside of \ref FffGcodeWriter::processLayer
	 *
	 * Depending on whether we need to prime on the first layer, or anywhere in the print,
	 * the layer numbers are all zero (or less in case of raft)
	 * or they are the first layer at which the extruder is needed
	 */
	int extruder_prime_layer_nr[16];

	std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder; //!< The settings used relating to minimal layer time and fan speeds. Configured for each extruder.

	FffGcodeWriter();

	/*!
	 * Set the target to write gcode to: to a file.
	 *
	 * Used when CuraEngine is used as command line tool.
	 *
	 * \param filename The filename of the file to which to write the gcode.
	 */
	bool setTargetFile(const char* filename)
	{
		printf("the program is at line 80 of the fffgcodewriter.h \n");
		output_file.open(filename);
		printf("the program is at line 82 of the fffgcodewriter.h \n");
		if (output_file.is_open())
		{
			gcode.setOutputStream(&output_file);
			return true;
		}
		return false;
	}

	/*!
	 * Get the total extruded volume for a specific extruder in mm^3
	 *
	 * Retractions and unretractions don't contribute to this.
	 *
	 * \param extruder_nr The extruder number for which to get the total netto extruded volume
	 * \return total filament printed in mm^3
	 */
	double getTotalFilamentUsed(size_t extruder_nr)
	{
		return gcode.getTotalFilamentUsed(extruder_nr);
	}

	/*!
	* Get the total estimated print time in seconds for each feature
	*
	* \return total print time in seconds for each feature

	std::vector<double> getTotalPrintTimePerFeature()
	{
		return gcode.getTotalPrintTimePerFeature();
	}
	 */
	 /*!
	  * Write all the gcode for the current meshgroup.
	  * This is the primary function of this class.
	  *
	  * \param[in] storage The data storage from which to get the polygons to print and the areas to fill.
	  * \param timeKeeper The stop watch to see how long it takes for each of the stages in the slicing process.
	  */
	void writeGCode(SliceDataStorage& storage, bool start);

	/*!
	 * \brief Set the FffGcodeWriter::fan_speed_layer_time_settings by
	 * retrieving all settings from the global/per-meshgroup settings.
	 */
	void setConfigFanSpeedLayerTime();
	/*!
	 * Set the retraction config globally, per extruder and per mesh.
	 *
	 * \param[out] storage The data storage to which to save the configurations
	 */
	void setConfigRetraction(SliceDataStorage& storage);

	void setConfigWipe(SliceDataStorage& storage);

	/*!
	* Get the extruder with which to start the print.
	*
	* Generally this is the adhesion_extruder_nr, but in case the platform adhesion type is none,
	* the extruder with lowest number which is used on the first layer is used as initial extruder.
	*
	* \param[in] storage where to get settings from.
	*/
	unsigned int getStartExtruder(const SliceDataStorage& storage);

	/*!
	 * Set the infill angles and skin angles in the SliceDataStorage.
	 *
	 * These lists of angles are cycled through to get the infill angle of a specific layer.
	 *
	 * \param mesh The mesh for which to determine the infill and skin angles.
	 */
	void setInfillAndSkinAngles(SliceDataStorage& storage);

	/*!
	* Set temperatures for the initial layer. Called by 'processStartingCode' and whenever a new object is started at layer 0.
	*
	* \param[in] storage where the slice data is stored.
	* \param[in] start_extruder_nr The extruder with which to start the print.
	*/
	void processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr);

	/*!
	 * Set temperatures and perform initial priming.
	 *
	 * Write a stub header if CuraEngine is in command line tool mode. (Cause writing the header afterwards would entail moving all gcode down.)
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param[in] start_extruder_nr The extruder with which to start the print.
	 */
	void processStartingCode(const SliceDataStorage& storage, const size_t start_extruder_nr);

	/*!
	 * Move up and over the already printed meshgroups to print the next meshgroup.
	 *
	 * \param[in] storage where the slice data is stored.
	 */
	void processNextMeshGroupCode(const SliceDataStorage& storage);

	/*!
	 * Add raft layer plans onto the FffGcodeWriter::layer_plan_buffer
	 *
	 * \param[in,out] storage where the slice data is stored.
	 */
	//void processRaft(const SliceDataStorage& storage);

	/*!
	 * Convert the polygon data of a layer into a layer plan on the FffGcodeWriter::layer_plan_buffer
	 *
	 * In case of negative layer numbers, create layers only containing the data from
	 * the helper parts (support etc) to fill up the gap between the raft and the model.
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param layer_nr The index of the layer to write the gcode of.
	 * \param total_layers The total number of layers.
	 * \return The layer plans
	 */
	LayerPlan& processLayer(SliceDataStorage& storage, int layer_nr, const size_t total_layers) const;

	/*!
	 * This function checks whether prime blob should happen for any extruder on the first layer.
	 * Priming will always happen, but the actual priming may or may not include a prime blob.
	 *
	 * Technically, this function checks whether any extruder needs to be primed (with a prime blob)
	 * separately just before they are used.
	 *
	 * \return whether any extruder need to be primed separately just before they are used
	 */
	bool getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, const size_t extruder_nr) const;

	/*!
	 * Plan priming of all used extruders which haven't been primed yet
	 * \param[in] storage where the slice data is stored.
	 * \param layer_plan The initial planning of the g-code of the layer.
	 */
	void ensureAllExtrudersArePrimed(const SliceDataStorage& storage, LayerPlan& layer_plan) const;

	/*!
	 * Add the skirt or the brim to the layer plan \p gcodeLayer if it hasn't already been added yet.
	 *
	 * This function should be called for only one layer;
	 * calling it for multiple layers results in the skirt/brim being printed on multiple layers.
	 *
	 * \param storage where the slice data is stored.
	 * \param gcodeLayer The initial planning of the g-code of the layer.
	 * \param extruder_nr The extruder train for which to process the skirt or
	 * brim.
	 */
	void processSkirtBrim(SliceDataStorage& storage, LayerPlan& gcodeLayer, unsigned int extruder_nr) const;
	bool processIroning(const SliceLayer& part, LayerPlan& gcode_layer) const;
	bool processInsets(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const size_t extruder_nr, const SliceLayerPart& part) const;
	/*!
	* Calculate in which order to plan the extruders for each layer
	* Store the order of extruders for each layer in extruder_order_per_layer for normal layers
	* and the order of extruders for raft/filler layers in extruder_order_per_layer_negative_layers.
	*
	* Only extruders which are (most probably) going to be used are planned
	*
	* \note At the planning stage we only have information on areas, not how those are filled.
	* If an area is too small to be filled with anything it will still get specified as being used with the extruder for that area.
	*
	* Computes \ref FffGcodeWriter::extruder_prime_layer_nr, \ref FffGcodeWriter::extruder_order_per_layer and \ref FffGcodeWriter::extruder_order_per_layer_negative_layers
	*
	* \param[in] storage where the slice data is stored.
	*/
	void calculateExtruderOrderPerLayer(const SliceDataStorage& storage);

	/*!
	 * Gets a list of extruders that are used on the given layer, but excluding the given starting extruder.
	 * When it's on the first layer, the prime blob will also be taken into account.
	 *
	 * \note At the planning stage we only have information on areas, not how those are filled.
	 * If an area is too small to be filled with anything it will still get specified as being used with the extruder for that area.
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param current_extruder The current extruder with which we last printed
	 * \return The order of extruders for a layer beginning with \p current_extruder
	 */
	//std::vector<size_t> getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const size_t start_extruder, const int& layer_nr) const;

	/*!
	 * Calculate in which order to plan the meshes of a specific extruder
	 * Each mesh which has some feature printed with the extruder is included in this order.
	 * One mesh can occur in the mesh order of multiple extruders.
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param extruder_nr The extruder for which to determine the order
	 * \return A vector of mesh indices ordered on print order for that extruder.
	 */
	//std::vector<size_t> calculateMeshOrder(const SliceDataStorage& storage, const size_t extruder_nr) const;

	/*!
	 * Add a single layer from a single mesh-volume to the layer plan \p gcodeLayer in mesh surface mode.
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param mesh The mesh to add to the layer plan \p gcodeLayer.
	 * \param mesh_config the line config with which to print a print feature
	 * \param gcodeLayer The initial planning of the gcode of the layer.
	 */
	 // void addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcodeLayer) const;

	  /*!
	   * Add the open polylines from a single layer from a single mesh-volume to the layer plan \p gcodeLayer for mesh the surface modes.
	   *
	   * \param[in] storage where the slice data is stored.
	   * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
	   * \param mesh_config the line config with which to print a print feature
	   * \param gcodeLayer The initial planning of the gcode of the layer.
	   */
	   // void addMeshOpenPolyLinesToGCode(const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const;

		/*!
		 * Add all features of a given extruder from a single layer from a single mesh-volume to the layer plan \p gcode_layer.
		 *
		 * This adds all features (e.g. walls, skin etc.) of this \p mesh to the gcode which are printed using \p extruder_nr
		 *
		 * \param[in] storage where the slice data is stored.
		 * \param mesh The mesh to add to the layer plan \p gcode_layer.
		 * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
		 * \param mesh_config the line config with which to print a print feature
		 * \param gcode_layer The initial planning of the gcode of the layer.
		 */
	void addMeshLayerToGCode(const SliceDataStorage& storage, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const;

	/*!
	 * Add all features of the given extruder from a single part from a given layer of a mesh-volume to the layer plan \p gcode_layer.
	 * This only adds the features which are printed with \p extruder_nr.
	 *
	 * \param[in] storage where the slice data is stored.
	 * \param storage Storage to get global settings from.
	 * \param mesh The mesh to add to the layer plan \p gcode_layer.
	 * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
	 * \param mesh_config the line config with which to print a print feature
	 * \param part The part to add
	 * \param gcode_layer The initial planning of the gcode of the layer.
	 */
	void addMeshPartToGCode(const SliceDataStorage& storage, const size_t extruder_nr, const SliceLayerPart& part, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const;

	/*!
	 * \brief Add infill for a given part in a layer plan.
	 *
	 * \param gcodeLayer The initial planning of the gcode of the layer.
	 * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
	 * \param extruder_nr The extruder for which to print all features of the
	 * mesh which should be printed with this extruder.
	 * \param mesh_config the line config with which to print a print feature.
	 * \param part The part for which to create gcode.
	 * \return Whether this function added anything to the layer plan.
	 */
	bool processInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

	/*!
	 * \brief Add thicker (multiple layers) sparse infill for a given part in a
	 * layer plan.
	 *
	 * \param gcodeLayer The initial planning of the gcode of the layer.
	 * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
	 * \param extruder_nr The extruder for which to print all features of the
	 * mesh which should be printed with this extruder.
	 * \param mesh_config The line config with which to print a print feature.
	 * \param part The part for which to create gcode.
	 * \return Whether this function added anything to the layer plan.
	 */
	bool processMultiLayerInfill(const SliceDataStorage& storage, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcodeLayer, const size_t extruder_nr,  const SliceLayerPart& part) const;

	/*!
	 * \brief Add normal sparse infill for a given part in a layer.
	 * \param gcodeLayer The initial planning of the gcode of the layer.
	 * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
	 * \param extruder_nr The extruder for which to print all features of the
	 * mesh which should be printed with this extruder
	 * \param mesh_config The line config with which to print a print feature.
	 * \param part The part for which to create gcode.
	 * \return Whether this function added anything to the layer plan.
	 */
	bool processSingleLayerInfill(const SliceDataStorage& storage, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer, const size_t extruder_nr, const SliceLayerPart& part) const;

	void finalize();

	/*!
	* Calculate for each layer the index of the vertex that is considered to be the seam
	* \param storage where the slice data is stored.
	* \param total_layers The total number of layers
	*/
	void findLayerSeamsForSpiralize(SliceDataStorage& storage, size_t total_layers);

	/*!
	 * Calculate the index of the vertex that is considered to be the seam for the given layer
	 * \param storage where the slice data is stored.
	 * \param mesh the mesh containing the layer of interest
	 * \param layer_nr layer number of the layer whose seam verted index is required
	 * \param last_layer_nr layer number of the previous layer
	 * \return layer seam vertex index
	 */
	 //	 unsigned int findSpiralizedLayerSeamVertexIndex(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int layer_nr, const int last_layer_nr);
	

};
#endif
