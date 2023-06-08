extends Node
class_name FeagiCache

# This script holds cached data for feagi, both directly and in processed forms

signal FullCorticalData_Updated(FullCorticalData: Dictionary)

####################################
####### FEAGI Direct Inputs ########
####################################
# These vars come directly from FEAGI (with minimal cleanup processing)

######### Directly Usable ##########
var pns_current_IPU: Dictionary:
	set(v): _pns_current_IPU = v
	get: return _pns_current_IPU
var pns_current_OPU: Dictionary:
	set(v): _pns_current_OPU = v
	get: return _pns_current_OPU

var genome_areaIDList: Dictionary:
	set(v): _genome_areaIDList = v
	get: return _genome_areaIDList
var genome_morphologyList: Array:
	set(v): _genome_morphologyList = v
	get: return _genome_morphologyList
var genome_fileName: String:
	set(v): _genome_fileName = v
	get: return _genome_fileName
var genome_corticalAreaIDList: Array:
	set(v): _genome_corticalAreaIDList = v; FCD_CorticalIDListRdy = true; Update_FullCorticalData()
	get: return _genome_corticalAreaIDList
var genome_corticalAreaNameList: Array:
	set(v): _genome_corticalAreaNameList = v
	get: return _genome_corticalAreaNameList
var genome_cortical_id_name_mapping: Dictionary:
	set(v): _genome_cortical_id_name_mapping = v; FCD_CorticalNameDictRdy = true; Update_FullCorticalData()
	get: return _genome_cortical_id_name_mapping
var circuit_list: Array:
	set(v): _circuit_list = v;
	get: return _circuit_list
var circuit_size: Array:
	set(v): _circuit_size = v;
	get: return _circuit_size

var connectome_properties_mappings: Dictionary:
	set(v): _connectome_properties_mappings = v; FCD_ConnectomePropertiesMappings = true; Update_FullCorticalData()
	get: return _connectome_properties_mappings

var burst_rate: float:
	set(v): _burst_rate = v
	get: return _burst_rate

######### Internal Caching #########
var _pns_current_IPU: Dictionary
var _pns_current_OPU: Dictionary

var _genome_areaIDList: Dictionary
var _genome_morphologyList: Array
var _genome_fileName: String
var _genome_corticalAreaIDList: Array
var _genome_corticalAreaNameList: Array
var _genome_cortical_id_name_mapping: Dictionary
var _circuit_list : Array
var _circuit_size : Array

var _connectome_properties_mappings: Dictionary

var _burst_rate: float

###### Other Internal Values #######
var _allConnectionReferencess: Array # IDs used to connect cortexes to each other

####################################
###### FEAGI Processed Inputs ######
####################################
# These vars have been processed, often because they have multiple dependencies



var FCD_CorticalIDListRdy = false; var FCD_CorticalNameDictRdy = false; var FCD_ConnectomePropertiesMappings = false
var fullCorticalData := {}
func Update_FullCorticalData(): # Update an easy to use dictionary with mappings easily set up
	# check if prerequisites are ready to go
	if(!FCD_CorticalIDListRdy): return
	if(!FCD_CorticalNameDictRdy): return
	if(!FCD_ConnectomePropertiesMappings): return
	# prereqs passed, reset them and continue
	FCD_CorticalIDListRdy = false; FCD_CorticalNameDictRdy = false; FCD_ConnectomePropertiesMappings = false
	fullCorticalData = InitMappingData(connectome_properties_mappings, genome_corticalAreaIDList, genome_cortical_id_name_mapping)
	FullCorticalData_Updated.emit(fullCorticalData)



####################################
######### Data Management ##########
####################################

# Convert Raw Connectome data from feagi to a dictionary structure usable by the node graph
# Dictionary {
# 	StringIDOfCortex:
#		{ "cortexReference": IntId (required by node Graph) - a randomized and unique int identifier for this cortex,
#		  "friendlyName: String,
#		  "connectionsIntIDs": [int array of connected cortexes, using their cortexReference],
#		  "connectionsStrIDs": [Str Array of connected cortexes, using the cortex IDs from FEAGI directly}
func InitMappingData(rawConnectomeMappings: Dictionary, orderedIDList: Array, FriendlyName_IDMapping: Dictionary) -> Dictionary:
	# lets abuse the fact that connectomeMappings includes a connectome list too!
	
	# clear old values
	_allConnectionReferencess = []
	
	# preinit to minimize garbage collection
	var cortexData := {}
	var newCortexRef: int
	var cortexReferenceArrCache: Array
	
	# 2 for loops is not particuarly efficient. Too Bad!
	
	# Generate base connection mapping without connections (we need to generate 
	# connection IDs for graphEdit connections)
	for i in range(orderedIDList.size()):
		newCortexRef = GenerateNewRandomIntForRef(_allConnectionReferencess)
		_allConnectionReferencess.append(newCortexRef)
		cortexData[orderedIDList[i]] = {
			"cortexReference": newCortexRef,
			"connectionsIntIDs": [],
			"connectionsStrIDs": [],
			"friendlyName": FriendlyName_IDMapping[orderedIDList[i]]}
	
	# add in the connections
	for key in cortexData.keys():
		cortexReferenceArrCache = []
		for connection in rawConnectomeMappings[key]:
			# we need to match the string ID with the int ID
				cortexReferenceArrCache.append(cortexData[connection]["cortexReference"]) 
		cortexData[key]["connectionsIntIDs"] = cortexReferenceArrCache
		cortexData[key]["connectionsStrIDs"] = rawConnectomeMappings[key]
	
	return cortexData

# Generate new ints to use as connection references
func GenerateNewRandomIntForRef(previousValues: Array) -> int:
	var randGen = RandomNumberGenerator.new()
	var randNum: int
	while true:
		# odds of collession are tiny but lets check anyways 
		randNum = randGen.randi()
		if randNum not in previousValues:
			break;
	return randNum

