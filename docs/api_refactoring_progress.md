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
- [ ] `/cortical_area/cortical_type_options`
- [ ] `/cortical_area/cortical_id_name_mapping`
- [ ] `/cortical_area/cortical_name_location`
- [ ] `/cortical_area/cortical_area_name_list`
- [ ] `/cortical_area/cortical_locations_2d`
- [ ] `/cortical_area/cortical_area/geometry`
- [ ] `/cortical_area/coord_2d`
- [ ] `/cortical_area/coord_3d`
- [ ] `/cortical_area/ipu`
- [ ] `/cortical_area/opu`
- [ ] `/cortical_area/cortical_map_detailed`
- [ ] `/cortical_area/cortical_visibility`
- [ ] `/cortical_area/suppress_cortical_visibility`
- [ ] `/cortical_area/multi/cortical_area_properties`
- [ ] `/cortical_area/multi/cortical_area`
- [ ] `/cortical_area/multi/cortical_area`
- [ ] `/cortical_area/neuron_count`
- [ ] `/cortical_area/reset`

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
- [ ] `/genome/upload/essential`
- [ ] `/genome/upload/file`
- [ ] `/genome/file_name`
- [ ] `/genome/upload/string`
- [ ] `/genome/download_region`
- [ ] `/genome/upload/file/edit`
- [ ] `/genome/defaults/files`
- [ ] `/genome/genome_number`
- [ ] `/genome/amalgamation_by_upload`
- [ ] `/genome/amalgamation_by_filename`
- [ ] `/genome/cortical_template`
- [ ] `/genome/circuits`
- [ ] `/genome/append-file`
- [ ] `/genome/deploy`

### Morphology Endpoints
- [x] `/morphology/*` (all endpoints now use CoreAPIService)

## Priority 3: Peripheral Operation Endpoints

- [ ] `/evolution/*`
- [ ] `/agent/*`
- [ ] `/insight/*`
- [ ] `/region/*`
- [ ] `/cortical_mapping/*`
- [ ] `/neuroplasticity/*`
- [ ] `/input/*`
- [ ] `/network/*`
- [ ] `/simulation/*`
- [ ] `/system/*`
- [ ] `/training/*`

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

### Remaining Work

1. **Complete Priority 1 Endpoints**
   - Finish refactoring cortical_area endpoints

2. **Address Priority 2 Endpoints**
   - Complete remaining genome endpoints
   
3. **Tackle Priority 3 Endpoints**
   - Refactor peripheral operation endpoints

4. **Testing**
   - Implement comprehensive tests for refactored endpoints
   - Verify all functionality works as expected 