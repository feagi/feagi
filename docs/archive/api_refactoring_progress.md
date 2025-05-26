# API Refactoring Progress

This document tracks the progress of the API refactoring effort to ensure all API endpoints use CoreAPIService instead of directly accessing ConnectomeManager.

## Priority 1: Core Operation Endpoints

### Burst Engine Endpoints
- [x] `/burst_engine/burst_counter`
- [x] `/burst_engine/config`
- [x] `/burst_engine/fcl_sampler/config`
- [x] `/burst_engine/fcl_sampler/area/{area_id}/sample_rate`
- [x] `/burst_engine/stats`
- [x] `/burst_engine/membrane_potentials`

### Connectome Endpoints
- [x] `/connectome/cortical_areas/list/summary`
- [x] `/connectome/cortical_areas/list/transforming`
- [x] `/connectome/cortical_areas/list/detailed`
- [x] `/connectome/cortical_info`
- [x] `/connectome/plasticity`
- [x] `/connectome/path`
- [x] `/connectome/snapshot`
- [x] `/connectome/download-cortical-area`
- [x] `/connectome/upload-cortical-area`
- [x] `/connectome/properties/dimensions`
- [x] `/connectome/stats/cortical/cumulative`
- [x] `/connectome/properties/mappings`
- [x] `/connectome/neurons/batch`
- [x] `/connectome/synapses/batch`

### Cortical Area Endpoints
- [x] `/cortical_area/cortical_area_properties`
- [x] `/cortical_area/cortical_area`
- [x] `/cortical_area/cortical_area_id_list`
- [x] `/cortical_area/cortical_area_index_list`
- [x] `/cortical_area/cortical_types`
- [x] `/cortical_area/cortical_type_options`
- [x] `/cortical_area/cortical_id_name_mapping`
- [x] `/cortical_area/cortical_name_location`
- [x] `/cortical_area/cortical_area_name_list`
- [x] `/cortical_area/cortical_locations_2d`
- [x] `/cortical_area/cortical_area/geometry`
- [x] `/cortical_area/coord_2d`
- [x] `/cortical_area/coord_3d`
- [x] `/cortical_area/ipu`
- [x] `/cortical_area/opu`
- [x] `/cortical_area/cortical_map_detailed`
- [x] `/cortical_area/cortical_visibility`
- [x] `/cortical_area/suppress_cortical_visibility`
- [x] `/cortical_area/multi/cortical_area_properties`
- [x] `/cortical_area/multi/cortical_area` (PUT)
- [x] `/cortical_area/multi/cortical_area` (DELETE)
- [x] `/cortical_area/neuron_count`
- [x] `/cortical_area/reset`

## Priority 2: Genome Management Endpoints

### Genome Endpoints
- [x] `/genome/reset`
- [x] `/genome/download`
- [x] `/genome/upload/barebones`
- [x] `/genome/amalgamation_by_payload`
- [x] `/genome/amalgamation_destination`
- [x] `/genome/amalgamation_history`
- [x] `/genome/amalgamation`
- [x] `/genome/amalgamation_cancellation`
- [x] `/genome/upload/essential`
- [x] `/genome/upload/file`
- [x] `/genome/file_name`
- [x] `/genome/upload/string`
- [x] `/genome/download_region`
- [x] `/genome/upload/file/edit`
- [x] `/genome/defaults/files`
- [x] `/genome/genome_number`
- [x] `/genome/amalgamation_by_upload`
- [x] `/genome/amalgamation_by_filename`
- [x] `/genome/cortical_template`
- [x] `/genome/circuits`
- [x] `/genome/append-file`
- [x] `/genome/deploy`

### Morphology Endpoints
- [x] `/morphology/*` (all endpoints now use CoreAPIService)

### Neuroplasticity Endpoints
- [x] `/neuroplasticity/status`
- [x] `/neuroplasticity/configure`
- [x] `/neuroplasticity/enable/{area_id}`
- [x] `/neuroplasticity/disable/{area_id}`
- [x] `/neuroplasticity/transforming`
- [x] `/neuroplasticity/plasticity_queue_depth` (GET)
- [x] `/neuroplasticity/plasticity_queue_depth` (PUT)

## Priority 3: Peripheral Operation Endpoints

- [x] `/evolution/*`
  - [x] `/evolution/generations` (GET) - Get details about all generations
  - [x] `/evolution/change_register` (GET) - Get evolution change register
- [ ] `/agent/*`
- [x] `/insight/*`
  - [x] `/insight/neurons/membrane_potential_status` (POST) - Get monitoring status
  - [x] `/insight/neurons/membrane_potential_set` (POST) - Enable/disable monitoring
  - [x] `/insight/neuron/synaptic_potential_status` (POST) - Get monitoring status
  - [x] `/insight/neuron/synaptic_potential_set` (POST) - Enable/disable monitoring
- [x] `/region/*`
  - [x] `/region/region` (POST) - Create a new brain region
  - [x] `/region/region` (PUT) - Update brain region properties
  - [x] `/region/region` (GET) - Get brain region properties
  - [x] `/region/region` (DELETE) - Delete a brain region
  - [x] `/region/region_and_members` (DELETE) - Delete region with members
  - [x] `/region/regions` (GET) - List all brain regions (summary)
  - [x] `/region/regions_members` (GET) - List all regions with members
  - [x] `/region/region_titles` (GET) - List all region titles
  - [x] `/region/change_cortical_area_region` (PUT) - Change area's region
  - [x] `/region/change_region_parent` (PUT) - Change region's parent
  - [x] `/region/relocate_members` (PUT) - Relocate region members
- [x] `/cortical_mapping/*`
  - [x] `/cortical_mapping/efferents` (POST) - Get outgoing connections from area
  - [x] `/cortical_mapping/afferents` (POST) - Get incoming connections to area
  - [x] `/cortical_mapping/cortical_mappings_by_name` (POST) - Get mappings by name
  - [x] `/cortical_mapping/cortical_mappings_detailed` (POST) - Get detailed mappings
  - [x] `/cortical_mapping/mapping_properties` (POST) - Get mapping properties
  - [x] `/cortical_mapping/mapping_properties` (PUT) - Update mapping properties
  - [x] `/cortical_mapping/cortical_map` (GET) - Get the full cortical map
  - [x] `/cortical_mapping/delete_suggested_mappings` (DELETE) - Remove mappings
- [x] `/input/*` 
  - [x] `/input/vision` (GET)
  - [x] `/input/vision` (POST)
- [ ] `/network/*`
- [ ] `/simulation/*`
- [ ] `/system/*`
- [ ] `/training/*`
- [ ] `/monitoring/*` (New router for system metrics and monitoring)

## Implementation Notes

### Key Architectural Changes

1. **Service Layer Isolation**
   - API endpoints no longer directly access ConnectomeManager
   - All interactions with brain state go through CoreAPIService

2. **State Manager Integration**
   - CoreAPIService maintains tight integration with the state manager
   - CoreAPIService properly handles data synchronization and caching

3. **Improved Error Handling**
   - Consistent error handling patterns implemented
   - Better error messages and HTTP status codes

4. **Documentation**
   - Added proper docstrings to all methods
   - Ensured route documentation is consistent with implementation

5. **Dependency System**
   - Created specialized dependency checks for different system states
   - Implement proper layering of dependencies
   - Added state-specific dependencies:
     - `check_active_genome`: Ensures a genome is loaded
     - `check_connectome_ready`: Ensures connectome is initialized
     - `check_burst_engine_running`: Ensures burst engine is operational
     - `check_genome_and_connectome`: Combined check for both genome and connectome
     - `check_fully_operational`: Comprehensive check for all systems running
     - `check_plasticity_enabled`: Verifies neuroplasticity is enabled
     - `check_io_system_ready`: Checks for IPU/OPU availability
     - `check_deployment_ready`: Ensures system is ready for genome deployment
     - `check_amalgamation_ready`: Verifies no amalgamation is pending
     - `check_cortical_area_exists`: Confirms a specific cortical area exists
     - `check_monitoring_available`: Ensures monitoring capabilities are available

6. **Peripheral System Integration**
   - Added dedicated CoreAPIService methods for peripheral operations
   - Implemented vision configuration through CoreAPIService methods
   - Added comprehensive cortical mapping functionality with proper error handling
   - Implemented brain region management through CoreAPIService
   - Integrated monitoring and insights functionality with proper service isolation

7. **Brain Region Management**
   - Implemented comprehensive brain region management through CoreAPIService
   - Added methods for creating, updating, and deleting brain regions
   - Added support for region hierarchies and parent-child relationships
   - Added region association management between cortical areas and regions
   - Implemented proper validation and error handling for region operations

8. **Cortical Mapping**
   - Added dedicated methods for retrieving afferent and efferent connections
   - Implemented mapping property management through CoreAPIService
   - Added support for mapping visualization and detailed mapping information
   - Implemented region-level suggested mapping support
   - Centralized mapping operations through consistent service interfaces

9. **Monitoring and Insights**
   - Added dedicated methods for membrane potential and synaptic potential monitoring
   - Implemented safe access to monitoring scopes through CoreAPIService
   - Added proper error handling for monitoring service dependencies (e.g., InfluxDB)
   - Centralized monitoring configuration in the service layer

### Remaining Work

1. **✅ Complete Priority 1 Endpoints**
   - All Burst Engine, Connectome, and Cortical Area endpoints have been refactored

2. **✅ Complete Priority 2 Endpoints**
   - All Genome Management, Morphology, and Neuroplasticity endpoints have been refactored
   - Added proper error handling and state validation

3. **In Progress: Priority 3 Endpoints**
   - Completed `/input/vision` endpoints
   - Completed all `/region/*` endpoints
   - Completed all `/cortical_mapping/*` endpoints
   - Completed all `/evolution/*` endpoints
   - Completed all `/insight/*` endpoints
   - Created new methods in CoreAPIService for vision, regions, cortical mappings, and monitoring
   - Next targets: agent system endpoints
   - Create new `/monitoring/*` router for system metrics (partial implementation completed) 