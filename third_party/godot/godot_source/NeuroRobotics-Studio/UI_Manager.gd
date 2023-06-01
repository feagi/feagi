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
var UI_Top_TopBar: Unit
var UI_LeftBar: Unit
var UI_createcorticalBar : Unit
var UI_CreateNeuronMorphology : Unit
var UI_ManageNeuronMorphology : Unit
var UI_MappingDefinition : Unit
var UI_GraphCore: GraphCore
var UI_CreateMorphology: Unit
var vectors_holder = []


#####################################
# Initialization

func Activate(langISO: String):
	
	# Initialize Vars
	currentLanguageISO = langISO #TODO Better language handling
	# Initialize UI
	
	# Initialize TopBar
	UI_Top_TopBar = SCENE_UNIT.instantiate()
	add_child(UI_Top_TopBar)
	var topBarDict = HelperFuncs.GenerateDefinedUnitDict("TOPBAR", currentLanguageISO)
	UI_Top_TopBar.Activate(topBarDict)
	UI_Top_TopBar.DataUp.connect(TopBarInput)
	
#	SpawnMappingDefinition()
	
	
	# Initialize GraphCore
	UI_GraphCore = $graphCore #TODO: this is very temporary
	UI_GraphCore.DataUp.connect(GraphEditInput)
	
	# Connect window size change function
	get_tree().get_root().size_changed.connect(WindowSizedChanged)
	
	
	Activated = true
	
#	print(UI_createcorticalBar.componentData) # Delete this when you are done.
# 	This print shows all node's ID.

####################################
####### Input Event Handling #######
####################################

signal DataUp(data: Dictionary)

######### Top Bar Control ##########
# These are all examples
func TopBarInput(data: Dictionary, _compRef, _unitRef):
#	print(JSON.stringify(data)) # useful for debugging
	if "CORTICALAREAS" in data.values():
		# Drop downs specifically can either be button inputs or dropdown changes,
		# verify this
		if "button" in data.keys():
			# Initialize popUpBar
			if not UI_createcorticalBar:
				SpawnCorticalCrete()
#				print(UI_createcorticalBar.componentData)
			else:
				UI_createcorticalBar.queue_free()
		#	UI_createcorticalBar.DataUp.connect(LeftBarInput)
			# button press
#			$".."/".."/Menu._on_add_pressed() #TODO: Need to change this approach. This is for example only
			print("Pressed Button!")
		else:
			# the cortical area drop down was changed
			var selectedCorticalArea: String = data["selected"]
			print("new selected area is " + selectedCorticalArea)
			
	elif "NEURONMORPHOLOGIES" in data.values():
	# Drop downs specifically can either be button inputs or dropdown changes,
	# verify this
		if "button" in data.keys():
			# button press
			if not UI_CreateMorphology:
				SpawnCreateMophology()
			else:
				UI_CreateMorphology.queue_free()
			print("Pressed Button!")
		else:
			# the cortical area drop down was changed
			var selectedCorticalArea: String = data["selected"]
			print("new selected area is " + selectedCorticalArea)
			if not UI_ManageNeuronMorphology:
				SpawnNeuronManager()
				print("composite data: ", UI_ManageNeuronMorphology.componentData)
			else:
				UI_ManageNeuronMorphology.queue_free()

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
#	print(JSON.stringify(data)) # useful for debugging
	if "CortexSelected" in data.keys():
		# Cortex has been selected, pop up side bar
		SpawnLeftBar()
		DataUp.emit(data)
		print("data: ", data)
	pass

# Is called whenever the game window size changes
func WindowSizedChanged():
	var viewPortSize: Vector2 = get_viewport_rect().size
	UI_GraphCore.size = viewPortSize
	#print(newWindowSize)

####################################
###### Relay Feagi Dependents ######
####################################

# Handles Recieving data from Core, and distributing it to the correct element
func RelayDownwards(callType, data) -> void:
	match(callType):
		REF.FROM.pns_current_ipu:
			pass
		REF.FROM.pns_current_opu:
			pass
		REF.FROM.genome_corticalAreaIdList:
			UI_Top_TopBar.RelayInputDataToComps({"CORTICALAREAS": {"options":data}})
		REF.FROM.genome_morphologyList:
			UI_Top_TopBar.RelayInputDataToComps({"NEURONMORPHOLOGIES": {"options":data}})
		REF.FROM.genome_fileName:
			UI_Top_TopBar.RelayInputDataToComps({"GENOMEFILENAME": {"label":data}})
		REF.FROM.connectome_properties_mappings:
			pass
		REF.FROM.godot_fullCorticalData:
			UI_GraphCore.RelayDownwards(REF.FROM.godot_fullCorticalData, data)
		REF.FROM.genome_corticalArea:
			
			# Data for Specific Cortical Area
			# Race conditions are technically possible. Verify input
			if UI_LeftBar == null: return # ignore if Leftbar isnt open
			#if data["cortical_id"] != UI_LeftBar.name: return # ignore if the correct sidebar isn't open
			
			# Assemble Dict to input values
			var inputVars = {
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
			}
			print(inputVars)
			UI_LeftBar.RelayInputDataToComps(inputVars)
			



####################################
############# Internals ############
####################################

func SpawnLeftBar():
	if UI_LeftBar != null:
		UI_LeftBar.queue_free()
	UI_LeftBar = SCENE_UNIT.instantiate()
	add_child(UI_LeftBar)
	var LeftBarDict = HelperFuncs.GenerateDefinedUnitDict("LEFTBAR", currentLanguageISO)
	UI_LeftBar.Activate(LeftBarDict)
	UI_LeftBar.DataUp.connect(LeftBarInput)
	var close = UI_LeftBar.get_child(0).get_child(1).get_child(0)
	var title = UI_LeftBar.get_child(0).get_child(0).get_child(0)
	var cortical_name = UI_LeftBar.get_child(2).get_child(1)
	var cortical_id = UI_LeftBar.get_child(3).get_child(1)
	var cortical_type = UI_LeftBar.get_child(4).get_child(1)
	var X = UI_LeftBar.get_child(5).get_child(0).get_child(1)
	var Y = UI_LeftBar.get_child(5).get_child(1).get_child(1)
	var Z = UI_LeftBar.get_child(5).get_child(2).get_child(1)
	var W = UI_LeftBar.get_child(6).get_child(0).get_child(1)
	var H = UI_LeftBar.get_child(6).get_child(1).get_child(1)
	var D = UI_LeftBar.get_child(6).get_child(2).get_child(1)
	var mem = UI_LeftBar.get_child(9).get_child(0).get_child(1)
	var syn = UI_LeftBar.get_child(9).get_child(1).get_child(1)
	close.connect("pressed", Callable(self,"close_button").bind(UI_LeftBar))
	title.text = $"../../Menu/cortical_menu/Control/name_string".text
	cortical_name.text = $"../../Menu/cortical_menu/Control/name_string".text
	cortical_id.text = $"../../Menu/cortical_menu/Control/cortical_id".text
#	cortical_type.set # This isnt even working, gotta see why
	X.value = $"../../Menu/cortical_menu/Control/X".value
	Y.value = $"../../Menu/cortical_menu/Control/Y".value
	Z.value = $"../../Menu/cortical_menu/Control/Z".value
	W.value = $"../../Menu/cortical_menu/Control/W".value
	H.value = $"../../Menu/cortical_menu/Control/H".value
	D.value = $"../../Menu/cortical_menu/Control/D".value
	mem.set_pressed($"../../Menu/button_choice/Control/mem".is_pressed())
	syn.set_pressed($"../../Menu/button_choice/Control/syn".is_pressed())
#	$"..".Update_GenomeCorticalArea_SPECIFC(cortical_id.text) ## Pass the raw id?
	
	
func SpawnCreateMophology():
	UI_CreateMorphology = SCENE_UNIT.instantiate()
	add_child(UI_CreateMorphology)
	var CMDict = HelperFuncs.GenerateDefinedUnitDict("CREATEMORPHOLOGY", currentLanguageISO)
	UI_CreateMorphology.Activate(CMDict)
	UI_CreateMorphology.DataUp.connect(CreateMorphologyInput)
	var close = UI_CreateMorphology.get_child(0).get_child(1).get_child(0)
	var button = UI_CreateMorphology.get_node("Unit_Vectors").get_node("Button_AddRowButton").get_child(0)
	var create_button = UI_CreateMorphology.get_node("Button_CreateButton").get_child(0)
	close.connect("pressed", Callable(self,"close_button").bind(UI_CreateMorphology))
	button.connect("pressed", Callable($Brain_Visualizer,"_morphology_add_row").bind("Vectors", UI_CreateMorphology.get_node("Unit_Vectors").get_node("Unit_XYZ"), UI_CreateMorphology.get_node("Unit_Vectors"), button, create_button))
#	_morphology_add_row
	
func SpawnCorticalCrete():
	UI_createcorticalBar = SCENE_UNIT.instantiate()
	add_child(UI_createcorticalBar)
	var createcorticalBar = HelperFuncs.GenerateDefinedUnitDict("CORTICAL_CREATE", currentLanguageISO)
	UI_createcorticalBar.Activate(createcorticalBar)
	UI_createcorticalBar.DataUp.connect(CorticalCreateInput)
#	var optionbutton = UI_createcorticalBar.get_child(1).get_child(0)
	var str_array = []
	for i in $".."/".."/Menu/addition_menu/OptionButton.item_count:
		str_array.append($".."/".."/Menu/addition_menu/OptionButton.get_item_text(i))
	UI_createcorticalBar.RelayInputDataToComps({"CORTICALAREA": {"options": (str_array)}})
#	if UI_createcorticalBar.DataUp.is_connected():
#	UI_createcorticalBar.DataUp.disconnect()
#	4.x - emitting_node.signal_name.disconnect(receiving_node.callback_function)
	var update = UI_createcorticalBar.get_child(7).get_child(0)
	var whd = UI_createcorticalBar.get_child(5)
	var xyz = UI_createcorticalBar.get_child(6)
	var close = UI_createcorticalBar.get_child(0).get_child(1).get_child(0)
	var name_input = UI_createcorticalBar.get_child(2).get_child(0).get_child(1)
	var optionlist = UI_createcorticalBar.get_child(1).get_child(1)
	var w = whd.get_child(0).get_child(1)
	var h = whd.get_child(1).get_child(1)
	var d = whd.get_child(2).get_child(1)
	var x = xyz.get_child(0).get_child(1)
	var y = xyz.get_child(1).get_child(1)
	var z = xyz.get_child(2).get_child(1)
	close.connect("pressed", Callable(self,"close_button").bind(UI_createcorticalBar))
	w.connect("value_changed",Callable($Brain_Visualizer,"_on_W_Spinbox_value_changed").bind([w,h,d,x,y,z]))
	h.connect("value_changed",Callable($Brain_Visualizer,"_on_H_Spinbox_value_changed").bind([w,h,d,x,y,z]))
	d.connect("value_changed",Callable($Brain_Visualizer,"_on_D_Spinbox_value_changed").bind([w,h,d,x,y,z]))
	x.connect("value_changed",Callable($Brain_Visualizer,"_on_X_SpinBox_value_changed").bind([w,h,d,x,y,z]))
	y.connect("value_changed",Callable($Brain_Visualizer,"_on_Y_Spinbox_value_changed").bind([w,h,d,x,y,z]))
	z.connect("value_changed",Callable($Brain_Visualizer,"_on_Z_Spinbox_value_changed").bind([w,h,d,x,y,z]))
	name_input.connect("text_changed",Callable($".."/Button_to_Autoload,"_on_type_text_changed"))
	update.connect("pressed",Callable($Brain_Visualizer,"_on_add_pressed").bind([w,h,d,x,y,z, name_input, optionlist, update]))

#func SpawnNeuronMorphology():
#	UI_CreateNeuronMorphology = SCENE_UNIT.instantiate()
#	add_child(UI_CreateNeuronMorphology)
#	var createmurphology = HelperFuncs.GenerateDefinedUnitDict("CREATE_MORPHOLOGY", currentLanguageISO)
#	UI_CreateNeuronMorphology.Activate(createmurphology)
#	UI_CreateNeuronMorphology.DataUp.connect(LeftBarInput)

func close_button(node):
	node.queue_free()
	
func add_row(node, holder):
	var new_node = node.get_node("Unit_Vectors").duplicate()
	var length_holder = len(holder)
	node.add_child(new_node)
	new_node.position.y = node.position.y + (20 * length_holder)

func SpawnNeuronManager():
	UI_ManageNeuronMorphology=SCENE_UNIT.instantiate()
	add_child(UI_ManageNeuronMorphology)
	var cerateneuronmorphology = HelperFuncs.GenerateDefinedUnitDict("MANAGE_MORPHOLOGY", currentLanguageISO)
	UI_ManageNeuronMorphology.Activate(cerateneuronmorphology)

func SpawnMappingDefinition():
	UI_MappingDefinition=SCENE_UNIT.instantiate()
	add_child(UI_MappingDefinition)
	var mappingdef = HelperFuncs.GenerateDefinedUnitDict("MAPPING_DEFINITION", currentLanguageISO)
	UI_MappingDefinition.Activate(mappingdef)

# Static Config
const SCENE_UNIT: PackedScene = preload("res://UI/Units/unit.tscn")

# proxys for properties
var _currentLanguageISO: String 

