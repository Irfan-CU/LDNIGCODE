//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef LAYER_PLAN_H
#define LAYER_PLAN_H



#include <vector>	

#include "FanSpeedLayerTime.h"
#include "GcodeExport.h"
#include "PathOrderOptimizer.h"
#include "SpaceFillType.h"
#include "GCodePath.h"
#include "NozzelTempInsert.h"
#include "timeMaterialEstimate.h"
#include "PathConfigStorage.h"
#include "Optional.h"
#include "Polygon.h"

    class Comb;
    class LayerPlan; // forward declaration so that ExtruderPlan can be a friend
	class LayerPlanBuffer; // forward declaration so that ExtruderPlan can be a friend 
	class SliceDataStorage;
	class WallOverlapComputation;


	class ExtruderPlan
	{
		friend class LayerPlan; // TODO: LayerPlan still does a lot which should actually be handled in this class.
		friend class LayerPlanBuffer; // TODO: LayerPlanBuffer handles paths directly
	protected:
		std::vector<GCodePath> paths; //!< The paths planned for this extruder
		std::list<NozzleTempInsert> inserts; //!< The nozzle temperature command inserts, to be inserted in between paths

		double heated_pre_travel_time; //!< The time at the start of this ExtruderPlan during which the head travels and has a temperature of initial_print_temperature

		/*!
		 * The required temperature at the start of this extruder plan
		 * or the temp to which to heat gradually over the layer change between this plan and the previous with the same extruder.
		 *
		 * In case this extruder plan uses a different extruder than the last extruder plan:
		 * this is the temperature to which to heat and wait before starting this extruder.
		 *
		 * In case this extruder plan uses the same extruder as the previous extruder plan (previous layer):
		 * this is the temperature used to heat to gradually when moving from the previous extruder layer to the next.
		 * In that case no temperature (and wait) command will be inserted from this value, but a NozzleTempInsert is used instead.
		 * In this case this member is only used as a way to convey information between different calls of \ref LayerPlanBuffer::processBuffer
		 */
		double required_start_temperature;
		std::optional<double> extrusion_temperature; //!< The normal temperature for printing this extruder plan. That start and end of this extruder plan may deviate because of the initial and final print temp (none if extruder plan has no extrusion moves)
		std::optional<std::list<NozzleTempInsert>::iterator> extrusion_temperature_command; //!< The command to heat from the printing temperature of this extruder plan to the printing temperature of the next extruder plan (if it has the same extruder).
		std::optional<double> prev_extruder_standby_temp; //!< The temperature to which to set the previous extruder. Not used if the previous extruder plan was the same extruder.

		TimeMaterialEstimates estimates; //!< Accumulated time and material estimates for all planned paths within this extruder plan.

	public:
		size_t extruder_nr; //!< The extruder used for this paths in the current plan.

		/*!
		 * Simple contructor.
		 *
		 * \warning Doesn't set the required temperature yet.
		 *
		 * \param extruder The extruder number for which this object is a plan.
		 * \param layer_nr The layer index of the layer that this extruder plan is
		 * part of.
		 * \param is_raft_layer Whether this extruder plan is part of a raft layer.
		 */
		ExtruderPlan(const size_t extruder, const int layer_nr, const bool is_initial_layer, const bool is_raft_layer, const coord_tIrfan layer_thickness, const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config);
		double getExtrudeSpeedFactor();
		/*!
		 * Add a new Insert, constructed with the given arguments
		 *
		 * \see NozzleTempInsert
		 *
		 * \param contructor_args The arguments for the constructor of an insert
		 */
		template<typename... Args>

		void insertCommand(Args&&... contructor_args)
		{
			inserts.emplace_back(contructor_args...);
		}

		/*!
		 * Insert the inserts into gcode which should be inserted before \p path_idx
		 *
		 * \param path_idx The index into ExtruderPlan::paths which is currently being consider for temperature command insertion
		 * \param gcode The gcode exporter to which to write the temperature command.
		 */
		void handleInserts(unsigned int& path_idx, GCodeExport& gcode)
		{
			while (!inserts.empty() && path_idx >= inserts.front().path_idx)
			{ // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
				inserts.front().write(gcode);
				inserts.pop_front();
			}
		}

		/*!
		 * Insert all remaining temp inserts into gcode, to be called at the end of an extruder plan
		 *
		 * Inserts temperature commands which should be inserted _after_ the last path.
		 * Also inserts all temperatures which should have been inserted earlier,
		 * but for which ExtruderPlan::handleInserts hasn't been called correctly.
		 *
		 * \param gcode The gcode exporter to which to write the temperature command.
		 */
		

		void handleAllRemainingInserts(GCodeExport& gcode)
		{
			while (!inserts.empty())
			{ // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
				NozzleTempInsert& insert = inserts.front();
				assert(insert.path_idx == paths.size());
				insert.write(gcode);
				inserts.pop_front();
			}
		}

		void processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, curaIrfan::PointIrfan starting_position);

		/*!
		 * Applying speed corrections for minimal layer times and determine the fanSpeed.
		 *
		 * \param force_minimal_layer_time Whether we should apply speed changes and perhaps a head lift in order to meet the minimal layer time
		 * \param starting_position The position the head was before starting this extruder plan																													  
		 */
		//void processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, curaIrfan::PointIrfan starting_position);

		/*!
		 * Set the extrude speed factor. This is used for printing slower than normal.
		 *
		 * Leaves the extrusion speed as is for values of 1.0
		 *
		 * \param speedFactor The factor by which to alter the extrusion move speed
		 */
		//void setExtrudeSpeedFactor(const double speed_factor);

		/*!
		 * Get the extrude speed factor. This is used for printing slower than normal.
		 *
		 * \return The factor by which to alter the extrusion move speed
		 */
		//double getExtrudeSpeedFactor();

		/*!
		 * Set the travel speed factor. This is used for performing non-extrusion travel moves slower than normal.
		 *
		 * Leaves the extrusion speed as is for values of 1.0
		 *
		 * \param speedFactor The factor by which to alter the non-extrusion move speed
		 */
		//void setTravelSpeedFactor(double speed_factor);

		/*!
		 * Get the travel speed factor. This is used for travelling slower than normal.
		 *
		 * Limited to at most 1.0
		 *
		 * \return The factor by which to alter the non-extrusion move speed
		 */
		//double getTravelSpeedFactor();

		/*!
		 * Get the fan speed computed for this extruder plan
		 *
		 * \warning assumes ExtruderPlan::processFanSpeedAndMinimalLayerTime has already been called
		 *
		 * \return The fan speed computed in processFanSpeedAndMinimalLayerTime
		 */
		double getFanSpeed();

	protected:
		int layer_nr; //!< The layer number at which we are currently printing.
		bool is_initial_layer; //!< Whether this extruder plan is printed on the very first layer (which might be raft)
		const bool is_raft_layer; //!< Whether this is a layer which is part of the raft

		coord_tIrfan layer_thickness; //!< The thickness of this layer in Z-direction

		const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings; //!< The fan speed and layer time settings used to limit this extruder plan

		const RetractionConfig& retraction_config; //!< The retraction settings for the extruder of this plan

		double extrudeSpeedFactor; //!< The factor by which to alter the extrusion move speed

		double extraTime; //!< Extra waiting time at the and of this extruder plan, so that the filament can cool
		double totalPrintTime; //!< The total naive time estimate for this extruder plan

		double fan_speed; //!< The fan speed to be used during this extruder plan

		/*!
		 * Set the fan speed to be used while printing this extruder plan
		 *
		 * \param fan_speed The speed for the fan
		 */
		//void setFanSpeed(double fan_speed);

		/*!
		 * Force the minimal layer time to hold by slowing down and lifting the head if required.
		 *
		 */
		void forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrusionTime);

		/*!
		 * Compute naive time estimates (without accounting for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
		 * and store them in each ExtruderPlan and each GCodePath.
		 *
		 * \param starting_position The position the head was in before starting this layer
		 * \return the total estimates of this layer
		 */
	//	TimeMaterialEstimates computeNaiveTimeEstimates(curaIrfan::PointIrfan starting_position);
	};
	
	class LayerPlanBuffer; // forward declaration to prevent circular dependency
	
	class LayerPlan
	{
		friend class LayerPlanBuffer;
		friend class SliceLayerPart;

	private:
		const SliceDataStorage& storage;
	public:
		const PathConfigStorage configs_storage;
		coord_tIrfan z;
		coord_tIrfan final_travel_z;
		bool mode_skip_agressive_merge;
		void setIsInside(bool _is_inside);
		int layer_mat;
		std::vector <SliceLayerPart> layer_parts;
		std::vector <int> layer_parts_mat;
		 
	private:
		Polygons computeCombBoundaryInside(const size_t max_inset);
		const int layer_nr; //!< The layer number of this layer plan
		const bool is_initial_layer; //!< Whether this is the first layer (which might be raft)
		const bool is_raft_layer; //!< Whether this is a layer which is part of the raft
		coord_tIrfan layer_thickness;
		Comb* comb;
		std::vector<curaIrfan::PointIrfan> layer_start_pos_per_extruder; //!< The starting position of a layer for each extruder
		std::optional<curaIrfan::PointIrfan> last_planned_position; //!< The last planned XY position of the print head (if known)
		
		std::string current_mesh; //<! A unique ID for the mesh of the last planned move.

		/*!
		 * Whether the skirt or brim polygons have been processed into planned paths
		 * for each extruder train.
		 */
		
		bool skirt_brim_is_processed[2];

		std::vector<ExtruderPlan> extruder_plans; //!< should always contain at least one ExtruderPlan

		size_t last_extruder_previous_layer; //!< The last id of the extruder with which was printed in the previous layer
		ExtruderTrain* last_planned_extruder;
		std::optional<curaIrfan::PointIrfan> first_travel_destination; //!< The destination of the first (travel) move (if this layer is not empty)
		bool first_travel_destination_is_inside; //!< Whether the destination of the first planned travel move is inside a layer part
		bool was_inside; //!< Whether the last planned (extrusion) move was inside a layer part
		bool is_inside; //!< Whether the destination of the next planned travel move is inside a layer part
		Polygons comb_boundary_inside1; //!< The minimum boundary within which to comb, or to move into when performing a retraction.
		Polygons comb_boundary_inside2; //!< The boundary preferably within which to comb, or to move into when performing a retraction.
		bool  processconfigs;
		coord_tIrfan comb_move_inside_distance;  //!< Whenever using the minimum boundary for combing it tries to move the coordinates inside by this distance after calculating the combing.
		Polygons bridge_wall_mask; //!< The regions of a layer part that are not supported, used for bridging
		Polygons overhang_mask; //!< The regions of a layer part where the walls overhang
		const std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder;
		// Parts in the layer 
		
		// Material in each part
		// Paths relation to the part
		


	private:
		/*!
		 * Either create a new path with the given config or return the last path if it already had that config.
		 * If LayerPlan::forceNewPathStart has been called a new path will always be returned.
		 *
		 * \param config The config used for the path returned
		 * \param space_fill_type The type of space filling which this path employs
		 * \param flow (optional) A ratio for the extrusion speed
		 * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
		 * \param speed_factor (optional) a factor which the speed will be multiplied by.
		 * \return A path with the given config which is now the last path in LayerPlan::paths
		 */
		GCodePath* getLatestPathWithConfig(coord_tIrfan layer_thickness, const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio flow = 1.0_r, bool spiralize = false, const Ratio speed_factor = 1.0_r);
		
	public:
		size_t getExtruder() const
		{
			return extruder_plans.back().extruder_nr;  //extruder nr is 0;
		}

		void forceNewPathStart();

		LayerPlan(const SliceDataStorage& storage, int layer_nr, coord_tIrfan z, coord_tIrfan layer_height, size_t start_extruder, const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, coord_tIrfan comb_boundary_offset, coord_tIrfan comb_move_inside_distance, coord_tIrfan travel_avoid_distance);
	
		~LayerPlan();

		ExtruderTrain* getLastPlannedExtruderTrain();
	 
		int getLayerNr()
		{
			return layer_nr;
		}

		curaIrfan::PointIrfan getLastPlannedPositionOrStartingPosition() const
		{
			return last_planned_position.value_or(layer_start_pos_per_extruder[getExtruder()]);
		}

		float extruder_offset;

		void writeGCode(GCodeExport& gcode);

		void setMesh(const std::string mesh_id);
		
		void planPrime();

		bool setExtruder(const size_t extruder_nr);
		
		void addLinesByOptimizer(coord_tIrfan layer_thickness , const GCodePathConfig& config, const Polygons& polygons, int layernum,int mat, SpaceFillType space_fill_type, bool enable_travel_optimization = false, int wipe_dist = 0, float flow_ratio = 1.0, std::optional<curaIrfan::PointIrfan> near_start_location = std::optional<curaIrfan::PointIrfan>(), double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT);
	        
		void addExtrusionMove(coord_tIrfan layer_thickness, const GCodePathConfig& config, curaIrfan::PointIrfan p, int layernum, int mat, SpaceFillType space_fill_type, const Ratio& flow = 1.0, bool spiralize = false, Ratio speed_factor = 1.0, double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT);
	 	
		bool getSkirtBrimIsPlanned(unsigned int extruder_nr) const
		{
			return skirt_brim_is_processed[extruder_nr];
		}

		std::optional<std::pair<curaIrfan::PointIrfan, bool>> getFirstTravelDestinationState() const;

		void setSkirtBrimIsPlanned(unsigned int extruder_nr)
		{
			skirt_brim_is_processed[extruder_nr] = true;
		}

		void moveInsideCombBoundary(int layernum, const coord_tIrfan distance);

		GCodePath& addTravel(coord_tIrfan layer_thickness, int layernum, curaIrfan::PointIrfan p, bool force_comb_retract = false);

		void processFanSpeedAndMinimalLayerTime(curaIrfan::PointIrfan starting_position);
		/*!
		 * Add a travel path to a certain point and retract if needed.
		 *
		 * No combing is performed.
		 *
		 * \param p The point to travel to
		 * \param path (optional) The travel path to which to add the point \p p
		 */
		GCodePath& addTravel_simple(int layer_nr, curaIrfan::PointIrfan p, GCodePath* path = nullptr);

		void optimizePaths(const curaIrfan::PointIrfan& starting_position);

		void addPolygon(coord_tIrfan layer_thickness, int layer_nr, ConstPolygonRef polygon, int start_Idx, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation = nullptr, coord_tIrfan wall_0_wipe_dist = 0, bool spiralize = false, const double& flow_ratio = 1.0, bool always_retract = false);
	
		void addWalls(const Polygons& walls, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, const ZSeamConfig& z_seam_config = ZSeamConfig(), coord_tIrfan wall_0_wipe_dist = 0, float flow_ratio = 1.0, bool always_retract = false);
		
		void addWall(ConstPolygonRef polygon, int start_idx, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, coord_tIrfan wall_0_wipe_dist, float flow_ratio, bool always_retract);

		void addWallLine(const curaIrfan::PointIrfan& p0, const curaIrfan::PointIrfan& p1, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, float flow, float& non_bridge_line_volume, Ratio speed_factor, double distance_to_bridge_start);

		void addPolygonsByOptimizer(coord_tIrfan layer_thickness, int layer_nr, const Polygons& polygons, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation = nullptr, const ZSeamConfig& z_seam_config = ZSeamConfig(), coord_tIrfan wall_0_wipe_dist = 0, bool spiralize = false, const double flow_ratio = 1.0, bool always_retract = false, bool reverse_order = false);

		unsigned locateFirstSupportedVertex(ConstPolygonRef wall, const unsigned start_idx) const;

		void setBridgeWallMask(const Polygons& polys)
		{
			bridge_wall_mask = polys;
		}
		void setOverhangMask(const Polygons& polys)
		{
			overhang_mask = polys;
		}
		
		bool ProcessCofigs_Storage();

		const Polygons* getCombBoundaryInside() const
		{
			return &comb_boundary_inside2;
		}

		
	};

#endif

