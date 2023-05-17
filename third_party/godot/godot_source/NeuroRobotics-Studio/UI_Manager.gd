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
var UI_GraphCore: GraphCore



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
	
	
	# Initialize GraphCore
	UI_GraphCore = $graphCore #TODO: this is very temporary
	UI_GraphCore.DataUp.connect(GraphEditInput)
	
	# Connect window size change function
	get_tree().get_root().size_changed.connect(WindowSizedChanged)
	
	
	Activated = true

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
			# button press
			$".."/".."/Menu._on_add_pressed() #TODO: Need to change this approach. This is for example only
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
			$Brain_Visualizer._on_Button_pressed() #TODO: Need to change this approach. This is for example only
			print("Pressed Button!")
		else:
			# the cortical area drop down was changed
			var selectedCorticalArea: String = data["selected"]
			print("new selected area is " + selectedCorticalArea)

######### Side Bar Control #########

func LeftBarInput(data: Dictionary, _compRef, _unitRef):
#	print(JSON.stringify(data)) # useful for debugging
	pass

############ Graph Edit ############

# Takes input from GraphEdit
func GraphEditInput(data: Dictionary):
#	print(JSON.stringify(data)) # useful for debugging
	if "CortexSelected" in data.keys():
		# Cortex has been selected, pop up side bar
		SpawnLeftBar()
		DataUp.emit(data)
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



# Static Config
const SCENE_UNIT: PackedScene = preload("res://UI/Units/unit.tscn")

# proxys for properties
var _currentLanguageISO: String 

