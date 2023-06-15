extends Node
# This script is mainly just to access enums. Not really even a script

# Structure: web path from feagi, where / is replaced with an underscore, and 
# existing underscores are replaced with camelCase

# Data points that are *from* Feagi in some way
enum FROM {
	burstEngine,
	
	pns_current_ipu, pns_current_opu, 
	
	genome_corticalAreaIdList, genome_corticalAreaNameList,
	genome_morphologyList, genome_fileName, genome_corticalArea,
	genome_cortical_id_name_mapping,
	circuit_list,
	circuit_size,
	connectome_properties_mappings,
	healthstatus,
	godot_fullCorticalData,
	OPULIST,
	IPULIST
}

enum TO {
	burstEngine
}

