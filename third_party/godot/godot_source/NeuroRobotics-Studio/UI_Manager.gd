extends Control
class_name UI_Manager

# This scripts handles UI instantiation, control, and feedback

#####################################

# Properties and Public Vars

var currentLanguageISO: String:
	get: return _currentLanguageISO
	set(v):
		_currentLanguageISO = v
		if(!Activated): return
		#TODO language changing code

var Activated: bool = false

# References
var UI_Top_TopBar: Newnit_Box
var UI_LeftBar: Newnit_Box
#var UI_createcorticalBar : Unit
#var UI_CreateNeuronMorphology : Unit
#var UI_ManageNeuronMorphology : Unit
#var UI_MappingDefinition : Unit
var UI_CircuitImport : Newnit_Box
var UI_GraphCore: GraphCore
#var UI_CreateMorphology: Unit
var UI_INDICATOR: Newnit_Box
var cache: FeagiCache
var vectors_holder = []
var data_holder = {} # to save data from API every call
var src_global 
var dst_global
var import_close_button

# Internal cached vars
var _sideBarChangedValues := {}

#####################################
# Initialization

func Activate(langISO: String):
	
	# Initialize Vars
	currentLanguageISO = langISO #TODO Better language handling
	# Initialize UI
	
	# Initialize TopBar
	var topBarDict = HelperFuncs.GenerateDefinedUnitDict("TOPBAR", currentLanguageISO)
	var createindicator = HelperFuncs.GenerateDefinedUnitDict("INDICATOR", currentLanguageISO)
	_SpawnTopBar(topBarDict)
	SpawnIndicator(createindicator)
	
	# Initialize GraphCore
	UI_GraphCore = $graphCore #TODO: this is very temporary
	UI_GraphCore.DataUp.connect(GraphEditInput)
	
	# Connect window size change function
	get_tree().get_root().size_changed.connect(WindowSizedChanged)
	
	
	Activated = true
	

func _SpawnTopBar(activation: Dictionary):
	UI_Top_TopBar = Newnit_Box.new()
	add_child(UI_Top_TopBar)
	UI_Top_TopBar.Activate(activation)
	UI_Top_TopBar.DataUp.connect(TopBarInput) # Amir, https://i.imgflip.com/10hpms.jpg
	var test = UI_Top_TopBar.GetReferenceByID("REFRESHRATE")
	var import_circuit = UI_Top_TopBar.GetReferenceByID("GENOMEFILENAME").get_node("sideButton_GENOMEFILENAME")
	var cortical_create = UI_Top_TopBar.GetReferenceByID("CORTICALAREAS").get_node("sideButton_CORTICALAREAS")
	import_circuit.connect("pressed", Callable($Brain_Visualizer,"_on_import_pressed"))
	

####################################
####### Input Event Handling #######
####################################

signal DataUp(data: Dictionary)

######### Top Bar Control ##########
# These are all examples
func TopBarInput(data: Dictionary, _compRef, _unitRef):
	print("FUnction called!")
	print(JSON.stringify(data)) # useful for debugging
	match(data["ID"]):
		"CORTICALAREAS":
			# Drop downs specifically can either be button inputs or dropdown changes,
			# verify this
			if "button" in data.keys():
				print("bwuk")
				# Initialize popUpBar
#				if not UI_createcorticalBar:
#					SpawnCorticalCrete()
#	#				print(UI_createcorticalBar.componentData)
#				else:
#					UI_createcorticalBar.queue_free()
			#	UI_createcorticalBar.DataUp.connect(LeftBarInput)
				# button press
	#			$".."/".."/Menu._on_add_pressed() #TODO: Need to change this approach. This is for example only
				print("Pressed Button!")
			else:
				# the cortical area drop down was changed
				var selectedCorticalArea: String = data["selected"]
				print("new selected area is " + selectedCorticalArea)
			
		"NEURONMORPHOLOGIES":
		# Drop downs specifically can either be button inputs or dropdown changes,
		# verify this
			if "button" in data.keys():
				# button press
#				if not UI_CreateMorphology:
#					SpawnCreateMophology()
#				else:
#					UI_CreateMorphology.queue_free()
				print("Pressed Button!")
			else:
				# the cortical area drop down was changed
				var selectedCorticalArea: String = data["selected"]
				print("new selected area is " + selectedCorticalArea)
#				if not UI_ManageNeuronMorphology:
#					SpawnNeuronManager()
#					print("composite data: ", UI_ManageNeuronMorphology.componentData)
#				else:
#					UI_ManageNeuronMorphology.queue_free()
		"REFRESHRATE":
			DataUp.emit({"updatedBurstRate": data["value"]})
			

func CreateMorphologyInput(data: Dictionary, _compRef: Node, _unitRef: Node):
	if "MorphologyType" == data["compID"]:
		#Drop down is changed, toggle between available morphology wizards
		var composite: Node = _unitRef.get_node("Unit_Composite")
		var patterns: Node = _unitRef.get_node("Unit_Patterns")
		var vectors: Node = _unitRef.get_node("Unit_Vectors")
		if vectors not in vectors_holder:
			vectors_holder.append(vectors)
		if data["selected"] == "Composite":
			composite.visibility = 0; patterns.visibility = 2; vectors.visibility = 2
		if data["selected"] == "Patterns":
			composite.visibility = 2; patterns.visibility = 0; vectors.visibility = 2
		if data["selected"] == "Vectors":
			composite.visibility = 2; patterns.visibility = 2; vectors.visibility = 0

#			vectors.get_node("Button_AddRowButton").get_child(0).connect("pressed", Callable(self,"add_row").bind(vectors_holder))
#			vectors.get_node("Unit_XYZ").get_child(0).get_child(1)
		else:
			composite.visibility = 2; patterns.visibility = 2; vectors.visibility = 2
######### Side Bar Control #########

func LeftBarInput(data: Dictionary, _compRef, _unitRef):
	print(JSON.stringify(data)) # useful for debugging
	match(data["ID"]):
		"UpdateButton":
			print("Update pressed!")
			# Push update to cortex
			# only push stuff that do not match what is cached
#			_sideBarChangedValues["cortical_id"] = UI_LeftBar.data["CorticalName"]
			$"..".Update_Genome_CorticalArea(_sideBarChangedValues)
			_sideBarChangedValues = {} # reset
		_:
			# Check if this is a neuron property, if so cache change for Update
			if _isNeuronProperty(data["ID"]):
				_sideBarChangedValues[data["ID"]] = data["value"]

func _isNeuronProperty(ID: String) -> bool:
	if ID == "VoxelNeuronDensity": return true
	if ID == "SynapticAttractivity": return true
	if ID == "PostSynapticPotential": return true
	if ID == "PSPMax": return true
	if ID == "PlasticityConstant": return true
	if ID == "FireThreshold": return true
	if ID == "Thresholdlimit": return true
	if ID == "RefactoryPeriod": return true
	if ID == "LeakConstant": return true
	if ID == "LeakVaribility": return true
	if ID == "ThresholdINC": return true
	if ID == "ConsecutiveFireCount": return true
	if ID == "SnoozePeriod": return true
	if ID == "DegeneracyConstant": return true
	return false



func CorticalCreateInput(data, _compRef, _unitRef):
	print("data inside corticalinput: ", data)
	if "CORTICALAREA" == data["compID"]:
		var textbox: Node = _unitRef.get_node("Unit_corticalnametext")
		var whd = _unitRef.get_node("Unit_WHD")
		var xyz = _unitRef.get_node("Unit_XYZ")
		var OPUIPU = _unitRef.get_node("Unit_OPUIPU")
		var downdrop = _unitRef.get_node("Unit_corticalnamedrop")
		if data["selected"] == "Custom":
			whd.visibility = 0
			textbox.visibility=0
			xyz.visibility = 0
			downdrop.visibility = 2
			OPUIPU.visibility = 2
		elif data["selected"] == "OPU" or data["selected"] == "IPU":
			whd.visibility = 2
			textbox.visibility = 2
			downdrop.visibility = 0
			downdrop.get_child(0).get_child(1).clear()
			if data["selected"] == "OPU":
				for i in $"../../Menu/addition_menu".opu_list:
					downdrop.get_child(0).get_child(1).add_item(i)
			if data["selected"] == "IPU":
				for i in $"../../Menu/addition_menu".ipu_list:
					downdrop.get_child(0).get_child(1).add_item(i)
			whd.visibility = 2
			xyz.visibility = 0
			OPUIPU.visibility = 0
		else:
			whd.visibility = 2
			xyz.visibility = 2
			whd.visibility = 2
			downdrop.visibility = 2
			textbox.visibility = 2
			OPUIPU.visibility = 2

############ Graph Edit ############

# Takes input from GraphEdit
func GraphEditInput(data: Dictionary):
	if "CortexSelected" in data.keys():
		# Cortex has been selected, pop up side bar
#		SpawnLeftBar(data["CortexSelected"])
		DataUp.emit(data)
	pass

# Is called whenever the game window size changes
func WindowSizedChanged():
	var viewPortSize: Vector2 = get_viewport_rect().size
	UI_GraphCore.size = viewPortSize

####################################
###### Relay Feagi Dependents ######
####################################

# Handles Recieving data from Core, and distributing it to the correct element
func RelayDownwards(callType, data) -> void:
	match(callType):
		REF.FROM.healthstatus:
			if UI_INDICATOR:
				print("data: ", data)
				if data["burst_engine"]:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator1": {"colorR": 0, "colorG": 255, "colorB": 0}}})
				else:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator1": {"colorR": 255, "colorG": 0, "colorB": 0}}})
				if data["genome_availability"]:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator2": {"colorR": 0, "colorG": 255, "colorB": 0}}})
				else:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator2": {"colorR": 255, "colorG": 0, "colorB": 0}}})
				if data["genome_validity"]:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator3": {"colorR": 0, "colorG": 255, "colorB": 0}}})
				else:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator3": {"colorR": 255, "colorG": 0, "colorB": 0}}})
				if data["brain_readiness"]:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator4": {"colorR": 0, "colorG": 255, "colorB": 0}}})
				else:
					UI_INDICATOR.SetData({"indicator_status": {"Indicator4": {"colorR": 255, "colorG": 0, "colorB": 0}}})

#		REF.FROM.circuit_size:
#			if UI_CircuitImport:
#				UI_CircuitImport.SetData({"WHD": {"W":{"value": data[0]}, "H":{"value": data[1]}, "D	":{"value": data[2]}}})
#		REF.FROM.circuit_list:
#			if UI_CircuitImport:
#				UI_CircuitImport.SetData({"dropdowncircuit": {"options":data}})
#		REF.FROM.pns_current_ipu:
#			pass
#		REF.FROM.pns_current_opu:
#			pass
		REF.FROM.genome_corticalAreaIdList:
			if UI_Top_TopBar:
				UI_Top_TopBar.SetData({"CORTICALAREAS": {"options":data}})
#			if UI_MappingDefinition:
#				UI_MappingDefinition.SetData({"testlabel": {"SOURCECORTICALAREA":{"options": data, "value": src_global}}})
#				UI_MappingDefinition.SetData({"testlabel": {"DESTINATIONCORTICALAREA":{"options": data, "value": dst_global}}})
		REF.FROM.genome_morphologyList:
			if UI_Top_TopBar:
				UI_Top_TopBar.SetData({"NEURONMORPHOLOGIES": {"options":data}})
#			if UI_MappingDefinition:
#				UI_MappingDefinition.SetData({"third_box": {"mappingdefinitions": {"options": data}}})
#				var original_dropdown = UI_MappingDefinition.get_node("Unit_third_box").get_node("DropDown_mappingdefinitions").get_node("OptionButton")
#				for i in UI_MappingDefinition.get_children():
#					if "Unit_third_box" in i.get_name():
#						for x in original_dropdown.get_item_count():
#							i.get_node("DropDown_mappingdefinitions").get_node("OptionButton").add_item(original_dropdown.get_item_text(x))
		REF.FROM.genome_fileName:
			UI_Top_TopBar.SetData({"GENOMEFILENAME": {"sideLabelText":data}})
#		REF.FROM.connectome_properties_mappings:
#			pass
#		REF.FROM.godot_fullCorticalData:
#			UI_GraphCore.RelayDownwards(REF.FROM.godot_fullCorticalData, data)
#		REF.FROM.genome_corticalArea:
#			# Data for Specific Cortical Area
#			# Race conditions are technically possible. Verify input
			if UI_LeftBar == null: return # ignore if Leftbar isnt open
			if data["cortical_id"] != UI_LeftBar.name: return # ignore if the correct sidebar isn't open
#
#			data_holder = data.duplicate()
#			# Assemble Dict to input values
			var inputVars = {
				"CorticalName": {"value": data["cortical_id"]},
				"CorticalID": {"value": data["cortical_name"]},
				"CorticalArea": {"value": data["cortical_group"]},
				"XYZ": {"Pos_X": {"value": data["cortical_coordinates"][0]}, "Pos_Y": {"value": data["cortical_coordinates"][1]}, "Pos_Z": {"value": data["cortical_coordinates"][2]}},
				"WHD": {"W": {"value": data["cortical_dimensions"][0]}, "H": {"value": data["cortical_dimensions"][1]}, "D": {"value": data["cortical_dimensions"][2]}},
				"VoxelNeuronDensity": {"value": data["cortical_neuron_per_vox_count"]},
				"SynapticAttractivity": {"value": data["cortical_synaptic_attractivity"]},
				"PostSynapticPotential": {"value": data["neuron_post_synaptic_potential"]},
				"PSPMax": {"value": data["neuron_post_synaptic_potential_max"]},
				"PlasticityConstant": {"value": data["neuron_plasticity_constant"]},
				"FireThreshold": {"value": data["neuron_fire_threshold"]},
				"RefactoryPeriod": {"value": data["neuron_refractory_period"]},
				"LeakConstant": {"value": data["neuron_leak_coefficient"]},
				"LeakVaribility": {"value": data["neuron_leak_variability"]},
				"ConsecutiveFireCount": {"value": data["neuron_consecutive_fire_count"]},
				"SnoozePeriod": {"value": data["neuron_snooze_period"]},
				"DegeneracyConstant": {"value": data["neuron_degeneracy_coefficient"]},
				"ChargeACC": {"value": data["neuron_mp_charge_accumulation"]},
				"PSPUNI": {"value": data["neuron_psp_uniform_distribution"]}
			}
#			#print(inputVars)
			UI_LeftBar.SetData(inputVars)
#			UI_LeftBar.SetData({"TITLEBAR": {"TITLE": {"label": data["cortical_id"]}}})
		REF.FROM.burstEngine:
			UI_Top_TopBar.SetData({"REFRESHRATE": {"value": data}})
	pass


####################################
############# Internals ############
####################################

func SpawnLeftBar(cortexName: String, activation: Dictionary):
	if UI_LeftBar != null:
		UI_LeftBar.queue_free() # We don't need this. We need to make it look prettier
	$"..".Update_GenomeCorticalArea_SPECIFC(cortexName) # Tell core to update cortex Info
	UI_LeftBar = Newnit_Box.new()
	add_child(UI_LeftBar)
	UI_LeftBar.Activate(activation)
	# Get available data with UI_LeftBar.data
#	UI_LeftBar.SetData({"TITLEBAR": {"TITLE": {"label": cortexName}}})

	
func mapping_definition_button(node):
#	var src_id = UI_LeftBar.get_node("Unit_TITLEBAR").get_node("Header_TITLE").get_node("Label").text
#	SpawnMappingDefinition(src_id, node.text)
#	Autoload_variable.BV_Core.Get_Morphology_information($Brain_Visualizer.name_to_id(node.text))
	pass
	
func SpawnCreateMophology():
#	UI_CreateMorphology = SCENE_UNIT.instantiate()
#	add_child(UI_CreateMorphology)
	var CMDict = HelperFuncs.GenerateDefinedUnitDict("CREATEMORPHOLOGY", currentLanguageISO)
#	UI_CreateMorphology.Activate(CMDict)
#	UI_CreateMorphology.DataUp.connect(CreateMorphologyInput)
#	var close = UI_CreateMorphology.get_child(0).get_child(1).get_child(0)
#	var button = UI_CreateMorphology.get_node("Unit_Vectors").get_node("Button_AddRowButton").get_child(0)
#	var create_button = UI_CreateMorphology.get_node("Button_CreateButton").get_child(0)
#	button.connect("pressed", Callable($Brain_Visualizer,"_morphology_add_row").bind("Vectors", UI_CreateMorphology.get_node("Unit_Vectors").get_node("Unit_XYZ"), UI_CreateMorphology.get_node("Unit_Vectors"), button, create_button))
	
#	_morphology_add_row
	
func SpawnCorticalCrete():
#	UI_createcorticalBar = SCENE_UNIT.instantiate()
#	add_child(UI_createcorticalBar)
	var createcorticalBar = HelperFuncs.GenerateDefinedUnitDict("CORTICAL_CREATE", currentLanguageISO)
#	UI_createcorticalBar.Activate(createcorticalBar)
#	UI_createcorticalBar.DataUp.connect(CorticalCreateInput)
#	var optionbutton = UI_createcorticalBar.get_child(1).get_child(0)
	var str_array = []
	for i in $".."/".."/Menu/addition_menu/OptionButton.item_count:
		str_array.append($".."/".."/Menu/addition_menu/OptionButton.get_item_text(i))
#	UI_createcorticalBar.SetData({"CORTICALAREA": {"options": (str_array)}})
#	if UI_createcorticalBar.DataUp.is_connected():
#	UI_createcorticalBar.DataUp.disconnect()
#	4.x - emitting_node.signal_name.disconnect(receiving_node.callback_function)
#	var update = UI_createcorticalBar.get_child(7).get_child(0)
#	var whd = UI_createcorticalBar.get_child(5)
#	var xyz = UI_createcorticalBar.get_child(6)
#	var close = UI_createcorticalBar.get_child(0).get_child(1).get_child(0)
#	var name_input = UI_createcorticalBar.get_child(2).get_child(0).get_child(1)
#	var optionlist = UI_createcorticalBar.get_child(1).get_child(1)
#	var w = whd.get_child(0).get_child(1)
#	var h = whd.get_child(1).get_child(1)
#	var d = whd.get_child(2).get_child(1)
#	var x = xyz.get_child(0).get_child(1)
#	var y = xyz.get_child(1).get_child(1)
#	var z = xyz.get_child(2).get_child(1)
#	w.connect("value_changed",Callable($Brain_Visualizer,"_on_W_Spinbox_value_changed").bind([w,h,d,x,y,z]))
#	h.connect("value_changed",Callable($Brain_Visualizer,"_on_H_Spinbox_value_changed").bind([w,h,d,x,y,z]))
#	d.connect("value_changed",Callable($Brain_Visualizer,"_on_D_Spinbox_value_changed").bind([w,h,d,x,y,z]))
#	x.connect("value_changed",Callable($Brain_Visualizer,"_on_X_SpinBox_value_changed").bind([w,h,d,x,y,z]))
#	y.connect("value_changed",Callable($Brain_Visualizer,"_on_Y_Spinbox_value_changed").bind([w,h,d,x,y,z]))
#	z.connect("value_changed",Callable($Brain_Visualizer,"_on_Z_Spinbox_value_changed").bind([w,h,d,x,y,z]))
#	name_input.connect("text_changed",Callable($"../../Button_to_Autoload","_on_type_text_changed"))
#	update.connect("pressed",Callable($Brain_Visualizer,"_on_add_pressed").bind([w,h,d,x,y,z, name_input, optionlist, update]))

#func SpawnNeuronMorphology():
#	UI_CreateNeuronMorphology = SCENE_UNIT.instantiate()
#	add_child(UI_CreateNeuronMorphology)
#	var createmurphology = HelperFuncs.GenerateDefinedUnitDict("CREATE_MORPHOLOGY", currentLanguageISO)
#	UI_CreateNeuronMorphology.Activate(createmurphology)
#	UI_CreateNeuronMorphology.DataUp.connect(LeftBarInput)

func SpawnIndicator(activation: Dictionary):
	UI_INDICATOR = Newnit_Box.new()
	add_child(UI_INDICATOR)
	UI_INDICATOR.Activate(activation)
	$"..".GET_health_status()
	
func add_row(node, holder):
	var new_node = node.get_node("Unit_Vectors").duplicate()
	var length_holder = len(holder)
	node.add_child(new_node)
	new_node.position.y = node.position.y + (20 * length_holder)
	
func SpawnCircuitImport(activation: Dictionary):
	UI_CircuitImport = Newnit_Box.new()
	add_child(UI_CircuitImport)
	UI_CircuitImport.Activate(activation)
	# Link to BV
#	var dropdown = UI_CircuitImport.get_node("DropDown_dropdowncircuit").get_node("OptionButton")
#	var x = UI_CircuitImport.get_node("Unit_XYZ").get_node("Counter_Pos_X").get_node("SpinBox")
#	var y = UI_CircuitImport.get_node("Unit_XYZ").get_node("Counter_Pos_Y").get_node("SpinBox")
#	var z = UI_CircuitImport.get_node("Unit_XYZ").get_node("Counter_Pos_Z").get_node("SpinBox")
#	var w = UI_CircuitImport.get_node("Unit_WHD").get_node("Counter_W").get_node("SpinBox")
#	var h = UI_CircuitImport.get_node("Unit_WHD").get_node("Counter_H").get_node("SpinBox")
#	var d = UI_CircuitImport.get_node("Unit_WHD").get_node("Counter_D").get_node("SpinBox")
#	var close_button = UI_CircuitImport.get_node("Unit_TITLEBAR").get_node("Button_CLOSEBUTTON").get_node("button")
#	var import_button = UI_CircuitImport.get_node("Button_IMPORTBUTTON").get_node("button")
#	import_button.connect("pressed", Callable($Brain_Visualizer,"_on_insert_button_pressed").bind([dropdown, x,y,z]))
#	close_button.connect("pressed", Callable($Brain_Visualizer,"_on_import_pressed"))
#	import_close_button = close_button
#	dropdown.connect("item_selected",Callable($Brain_Visualizer,"_on_ItemList_item_selected").bind(dropdown))
#	x.connect("value_changed",Callable($Brain_Visualizer,"_on_x_spinbox_value_changed").bind([x, y, z, w, h, d]))
#	y.connect("value_changed",Callable($Brain_Visualizer,"_on_y_spinbox_value_changed").bind([x, y, z, w, h, d]))
#	z.connect("value_changed",Callable($Brain_Visualizer,"_on_z_spinbox_value_changed").bind([x, y, z, w, h, d]))

func SpawnNeuronManager():
#	UI_ManageNeuronMorphology=SCENE_UNIT.instantiate()
#	add_child(UI_ManageNeuronMorphology)
	var cerateneuronmorphology = HelperFuncs.GenerateDefinedUnitDict("MANAGE_MORPHOLOGY", currentLanguageISO)
#	UI_ManageNeuronMorphology.Activate(cerateneuronmorphology)

func SpawnMappingDefinition(src, dst):
#	if UI_MappingDefinition:
#		UI_MappingDefinition.queue_free()
#		$Brain_Visualizer.plus_node.clear()
#	UI_MappingDefinition=SCENE_UNIT.instantiate()
#	add_child(UI_MappingDefinition)
	var mappingdef = HelperFuncs.GenerateDefinedUnitDict("MAPPING_DEFINITION", currentLanguageISO)
#	UI_MappingDefinition.Activate(mappingdef)
	$"..".Update_CortinalAreasIDs()
	var get_id_from_dst = $Brain_Visualizer.name_to_id(dst)
	src_global = src
	dst_global = get_id_from_dst
	var combine_url = '#&dst_cortical_area=$'.replace("#", src)
	combine_url= combine_url.replace("$", get_id_from_dst)
	Autoload_variable.BV_Core.Update_destination(combine_url)
	$"..".Update_MorphologyList()


# proxys for properties
var _currentLanguageISO: String 

