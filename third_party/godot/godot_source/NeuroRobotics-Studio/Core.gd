extends Node
class_name Core

####################################
####### Configuration & Setup ######
####################################
var FEAGI_RootAddress = str(network_setting.api_ip_address) + ":"+ str(network_setting.api_port_address)
@export var languageISO := "eng" #TODO proxy changes to UI manager

func _ready():
	# # # Build the bridge # # # 
	Autoload_variable.Core_BV = $GlobalUISystem/Brain_Visualizer
	Autoload_variable.Core_addOption = get_parent().get_node("Menu/Mapping_Properties/cortical_dropdown")
	Autoload_variable.Core_Menu = get_parent().get_node("Menu")
	Autoload_variable.Core_addition = get_parent().get_node("Menu/addition_menu")
	# Retrieve relvant Child Nodes
	NetworkAPI = $GlobalNetworkSystem
	UIManager = $GlobalUISystem
	FeagiCache = $FeagiCache
	# Connect Cache First
	FeagiCache.FullCorticalData_Updated.connect(Relay_fullCorticalData)
	
	# Activate Children
	UIManager.Activate(languageISO)
	UIManager.dataUp.connect(RetrieveEvents)
	
	# Lets pull latest info from FEAGI and trigger respective updates
	Update_CortinalAreasIDs()
	Update_MorphologyList()
	Update_GenomeFileName()
	Update_ConnectomeMappingReport()
	Update_CorticalAreaNameList()

####################################
####### Process From Below ########
####################################

# Respond to any events at the core level
func RetrieveEvents(data: Dictionary) -> void:
	if "CortexSelected" in data.keys():
		Update_GenomeCorticalArea_SPECIFC(data["CortexSelected"])

####################################
##### Update Feagi Dependents ######
####################################

# These are used to update certain elements from Feagi Directly

# The Godot Style guide can't stop me because I can't read!
# HTTP://docs.godotengine.org/en/stable/tutorials/scripting/gdscript/gdscript_styleguide.html#one-statement-per-line

# Run these to grab latest information from Feagi, and eventually trigger an update on all dependents
# WARNING: due to network latency, there is a delay between calling this and results appearing.
# Design your code with this in mind, it may be best to include any changes you desire in the _Relay function
func Update_IPUs(): Call_GET(ADD_GET_IPUList, _Relay_IPUs)
func Update_OPUs(): Call_GET(ADD_GET_OPUList, _Relay_OPUs)
func Update_CortinalAreasIDs(): Call_GET(ADD_GET_CorticalAreasIDs, _Relay_CorticalAreasIDs)
func Update_MorphologyList(): Call_GET(ADD_GET_MorphologyList, _Relay_MorphologyList)
func Update_GenomeFileName(): Call_GET(ADD_GET_GenomeFileName, _Relay_GenomeFileName)
func Update_ConnectomeMappingReport(): Call_GET(ADD_GET_ConnectomeMappingReport, _Relay_ConnectomeMappingReport)
func Update_CorticalAreaNameList(): Call_GET(ADD_GET_CorticalAreaNameList, _Relay_CorticalAreaNameList)
func Update_GenomeCorticalArea_SPECIFC(corticalArea: String): Call_GET(ADD_GET_Genome_CorticalArea, _Relay_GET_Genome_CorticalArea, corticalArea ) 
func Update_Dimensions(): Call_GET(ADD_GET_Dimensions, _Relay_Dimensions)
func Update_Refresh_Rate(): Call_GET(ADD_GET_Refresh_Rate, _Relay_Get_BurstRate)
func Update_Morphology_type(): Call_GET(ADD_GET_Morphology_types, _Relay_Morphology_type)
func Update_Cortical_grab_id(input): Call_GET(ADD_GET_cortical_id+input, _Relay_Cortical_grab_id)
func Update_Afferent_list(input): Call_GET(ADD_GET_Afferent+input, _Relay_Afferent)
func Update_Efferent_information(input): Call_GET(ADD_GET_Efferent+input, _Relay_Efferent)
func Get_Morphology_information(input): Call_GET(ADD_GET_Morphology_information+input, _Relay_Morphology_information)
func Update_destination(input): Call_GET(ADD_GET_update_destination+input, _Relay_Update_Destination)
func Get_circuit_list(): Call_GET(ADD_GET_circuit_list, _Relay_circuit_list)
func Get_mem_data(name: String): Call_GET(ADD_GET_mem+name, _Relay_Update_mem)
func Get_syn_data(name: String): Call_GET(ADD_GET_syn+name, _Relay_Update_syn)
func GET_OPU(name: String): Call_GET(ADD_OPU+name, _Relay_update_OPU)
func GET_IPU(name: String): Call_GET(ADD_IPU+name, _Relay_update_IPU)
func Update_BurstRate(newBurstRate: float):
	Call_POST(ADD_POST_BurstEngine, _Relay_ChangedBurstRate, {"burst_duration": newBurstRate})

func Update_cortical_area(input):
	Call_POST(ADD_POST_Add_Cortical, _Relay_updated_cortical, input)
	
func Update_custom_cortical_area(input):
	#using _Relay_updated_cortical since they both pass, thats it. leverage the same to save space
	Call_POST(ADD_POST_Add_Custom_Cortical, _Relay_updated_cortical, input)
	
func POST_Request_Brain_visualizer(url, dataIn):
	#using _Relay_updated_cortical since they both pass, thats it. leverage the same to save space
	Call_POST(url, _Relay_updated_cortical, dataIn)
	
func Update_Mapping_Properties(dataIn, extra_name =""): 
	Call_PUT(ADD_PUT_Mapping_Properties + extra_name, _Relay_PUT_Mapping_Properties, dataIn)

func PUT_Request_Brain_visualizer(url, dataIn): 
	Call_PUT(url, _Relay_PUT_BV_functions, dataIn)

func Delete_cortical_area(dataIn): 
	Call_DELETE(ADD_DELETE_remove_cortical_area + dataIn, _Relay_DELETE_Cortical_area)

func DELETE_Request_Brain_visualizer(url):
	Call_DELETE(url, _Relay_DELETE_Cortical_area)

func Update_Genome_CorticalArea(dataIn: Dictionary): Call_PUT(ADD_PUT_Genome_CorticalArea, _Relay_PUT_Genome_CorticalArea, dataIn)
####################################
###### Relay Feagi Dependents ######
####################################

####### From FEAGI Directly ########
# In this section, add any code that must be called when FEAGI updates these
# values
#TODO error handling

func _Relay_IPUs(_result, _response_code, _headers, body: PackedByteArray):
	# FEAGI updated IPUs
	FeagiCache.pns_current_IPU = JSON.parse_string(body.get_string_from_utf8())
	pass

func _Relay_OPUs(_result, _response_code, _headers, body: PackedByteArray):
	# FEAGI updated OPUs
	FeagiCache.pns_current_OPU = JSON.parse_string(body.get_string_from_utf8())
	pass

func _Relay_CorticalAreasIDs(_result, _response_code, _headers, body: PackedByteArray):
	# FEAGI updated Cortical Areas
	FeagiCache.genome_corticalAreaIDList = JSON.parse_string(body.get_string_from_utf8())
	UIManager.RelayDownwards(REF.FROM.genome_corticalAreaIdList, FeagiCache.genome_corticalAreaIDList)

func _Relay_ChangedBurstRate(_result, _response_code, _headers, _body: PackedByteArray):
	# FEAGI updated Burst Rate
	pass

func _Relay_updated_cortical(_result, _response_code, _headers, _body: PackedByteArray):
	pass
	
func _Relay_Get_BurstRate(_result, _response_code, _headers, body: PackedByteArray):
	Autoload_variable.Core_BV._on_get_burst_request_completed(_result, _response_code, _headers, body)

func _Relay_Dimensions(_result, _response_code, _headers, body: PackedByteArray):
	#Feagi Updated Dimensions
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	var create_json = {}
	if api_data != null:
		for i in api_data:
			create_json[i] = api_data[i]
		Godot_list.genome_data["genome"] = create_json

func _Relay_Morphology_type(_result, _response_code, _headers, body: PackedByteArray):
	#Feagi Updated Dimensions
	Autoload_variable.Core_BV._on_morphology_types_request_completed(_result, _response_code, _headers, body)
	
func _Relay_Cortical_grab_id(_result, _response_code, _headers, body: PackedByteArray):
	#Feagi Updated Dimensions
	Autoload_variable.Core_BV._on_HTTPRequest_request_completed(_result, _response_code, _headers, body)
	Autoload_variable.Core_BV._on_get_cortical_dst_request_completed(_result, _response_code, _headers, body)

func _Relay_Afferent(_result, _response_code, _headers, body: PackedByteArray):
	#Feagi Updated Dimensions
	Autoload_variable.Core_BV._on_afferent_request_completed(_result, _response_code, _headers, body)
	
func _Relay_MorphologyList(_result, _response_code, _headers, body: PackedByteArray):
	# FEAGI updated Morphology list
	FeagiCache.genome_morphologyList = JSON.parse_string(body.get_string_from_utf8())
	UIManager.RelayDownwards(REF.FROM.genome_morphologyList, FeagiCache.genome_morphologyList)
	if Autoload_variable.Core_BV.visible:
		Autoload_variable.Core_BV._on_morphology_list_request_completed(_result, _response_code, _headers, body)

func _Relay_GenomeFileName(_result, _response_code, _headers, body: PackedByteArray):
	var stringName := (body.get_string_from_utf8())
	stringName = stringName.replace('"', "") # Remove build in quotation marks
	stringName = stringName.left(-5) # remove the .json
	FeagiCache.genome_fileName =  stringName
	UIManager.RelayDownwards(REF.FROM.genome_fileName, stringName)
	if Autoload_variable.Core_BV.visible:
		Autoload_variable.Core_BV._on_get_genome_name_request_completed(_result, _response_code, _headers, body)

func _Relay_CorticalAreaNameList(_result, _response_code, _headers, body: PackedByteArray):
	# FEAGI updated Cortical Area Name list
	FeagiCache.genome_corticalAreaNameList = JSON.parse_string(body.get_string_from_utf8())
	UIManager.RelayDownwards(REF.FROM.genome_corticalAreaNameList, FeagiCache.genome_corticalAreaNameList)
	if get_parent().get_node("Menu").ready:
		Autoload_variable.Core_addOption._on_load_options_cortical_name_request_completed(_result, _response_code, _headers, body)
	
func _Relay_GET_Genome_CorticalArea(_result, _response_code, _headers, body: PackedByteArray):
	# Note, this is for a specific cortical Area
	var specificCortex = JSON.parse_string(body.get_string_from_utf8())
	# Hacky workaround since godot json gets confused on quotes
	for key in specificCortex.keys():
		if specificCortex[key] is int:
			specificCortex[key] = float(specificCortex[key])
	UIManager.RelayDownwards(REF.FROM.genome_corticalArea, specificCortex)

func _Relay_Morphology_information(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_type_rules_request_completed(_result, _response_code, _headers, _body)
	if get_parent().get_node("Menu/Mapping_Properties").visible:
		Autoload_variable.Core_BV._on_get_morphology_request_completed(_result, _response_code, _headers, _body)
func _Relay_Update_Destination(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_update_destination_info_request_completed(_result, _response_code, _headers, _body)
func _Relay_Update_mem(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_mem_request_request_completed(_result, _response_code, _headers, _body)
func _Relay_Update_syn(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_syn_request_request_completed(_result, _response_code, _headers, _body)
func _Relay_circuit_list(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_circuit_request_request_completed(_result, _response_code, _headers, _body)
func _Relay_update_OPU(_result, _response_code, _headers, _body):
	Autoload_variable.Core_addition._on_cortical_type_options_request_request_completed(_result, _response_code, _headers, _body)
func _Relay_update_IPU(_result, _response_code, _headers, _body):
	Autoload_variable.Core_addition._on_IPU_list_request_completed(_result, _response_code, _headers, _body)
	

func _Relay_Efferent(_result, _response_code, _headers, _body: PackedByteArray):
	Autoload_variable.Core_BV._on_information_button_request_completed(_result, _response_code, _headers, _body)

func _Relay_ConnectomeMappingReport(_result, _response_code, _headers, body: PackedByteArray):
	FeagiCache.connectome_properties_mappings = JSON.parse_string(body.get_string_from_utf8())

func _Relay_PUT_Genome_CorticalArea(_result, _response_code, _headers, _body: PackedByteArray):
	pass 

func _Relay_PUT_Mapping_Properties(_result, _response_code, _headers, _body: PackedByteArray):
	pass 

func _Relay_PUT_BV_functions(_result, _response_code, _headers, _body: PackedByteArray):
	pass 

func _Relay_DELETE_Cortical_area(_result, _response_code, _headers, _body):
	pass

##### Proxied from FEAGICache ######

# Convert Raw Connectome data from feagi to a dictionary structure usable by the node graph and other systems
# Dictionary {
# 	StringIDOfCortex:
#		{ "cortexReference": IntId (required by node Graph),
#		  "friendlyName: String
#		  "connections": [int array of connected cortexes, using their cortexReference]}
func Relay_fullCorticalData(dataIn: Dictionary):
	UIManager.RelayDownwards(REF.FROM.godot_fullCorticalData, dataIn)
	pass


####################################
############## Calls ###############
####################################

# TODO add error handling should network fail!
func Call_GET(address: String, proxiedFunction, stringToAppend: String = ""):
	NetworkAPI.Call(address + stringToAppend, HTTPClient.METHOD_GET, proxiedFunction)

func Call_POST(address: String, proxiedFunction, data2Send):
	NetworkAPI.Call(address, HTTPClient.METHOD_POST, proxiedFunction, data2Send)

func Call_PUT(address: String, proxiedFunction, data2Send):
	NetworkAPI.Call(address, HTTPClient.METHOD_PUT, proxiedFunction, data2Send)
	
func Call_DELETE(address: String, proxiedFunction):
	NetworkAPI.Call(address, HTTPClient.METHOD_DELETE, proxiedFunction)


####################################
############# Internals ############
####################################

# References
var NetworkAPI : SimpleNetworkAPI
var UIManager : UI_Manager
var FeagiCache: FeagiCache

# Get Requests
const SEC = "HTTP://"

var ADD_GET_IPUList:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/feagi/pns/current/ipu"
var ADD_GET_OPUList:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/feagi/pns/current/opu"
var ADD_GET_CorticalAreasIDs:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area_id_list"
var ADD_GET_MorphologyList:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/morphology_list"
var ADD_GET_GenomeFileName:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/file_name"
var ADD_GET_ConnectomeMappingReport:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/connectome/properties/mappings"
var ADD_GET_CorticalAreaNameList:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area_name_list"
var ADD_GET_Genome_CorticalArea:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area?cortical_area="
var ADD_GET_Afferent:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_mappings/afferents?cortical_area="
var ADD_GET_Efferent:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_mappings/efferents?cortical_area="
var ADD_GET_update_destination:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/mapping_properties?src_cortical_area="
var ADD_GET_circuit_list:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/circuits"
var ADD_GET_Dimensions:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/connectome/properties/dimensions"
var ADD_GET_Morphology_types:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/morphology_types"
var ADD_GET_Refresh_Rate:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/feagi/burst_engine/stimulation_period"
var ADD_GET_cortical_id:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area?cortical_area="
var ADD_GET_Morphology_information:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/morphology?morphology_name="
var ADD_GET_mem:
	get: return SEC + FEAGI_RootAddress + '/v1/feagi/monitoring/neuron/membrane_potential?cortical_area='
var ADD_GET_syn:
	get: return SEC + FEAGI_RootAddress + '/v1/feagi/monitoring/neuron/synaptic_potential?cortical_area='
var ADD_OPU:
	get: return SEC + FEAGI_RootAddress + '/v1/feagi/genome/cortical_type_options?cortical_type='
var ADD_IPU:
	get: return SEC + FEAGI_RootAddress + '/v1/feagi/genome/cortical_type_options?cortical_type='

# Post Requests
var ADD_POST_BurstEngine:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/feagi/burst_engine"
var ADD_POST_Add_Cortical:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area"
var ADD_POST_Add_Custom_Cortical:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/custom_cortical_area"

# Put Requests
var ADD_PUT_Genome_CorticalArea:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area"

var ADD_PUT_Mapping_Properties:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/mapping_properties"

	
# Delete Requests
var ADD_DELETE_remove_cortical_area:
	get: return SEC + FEAGI_RootAddress + "/v1/feagi/genome/cortical_area?cortical_area_name="
