extends Object
class_name HelperFuncs

# This script is full of commonly called functions. Static to decrease memory usage
# I will put my simple conditionals on one line, you cannot stop me!

# UI Directories
const PATH_UISTRUCTS = "res://UI_Structures/"
const PATH_LANGUAGES = "res://Language/"

# Default FallBack Language
const FALLBACK_LANG = "eng"

# Return dictionary key if exists, otherwise return default
static func GetIfCan(dict: Dictionary, key: String, default):
	var output
	if(dict.has(key)): output = dict[key]
	else: output = default
	return output

# Returns directory key. if it doesn't exist, force raise an error
static func MustGet(dict: Dictionary, key: String):
	assert(dict.has(key), "Key Missing from Input Dictionary")
	return dict[key]

# Returns a Unit initializer Dictionary from a structure Json, the given
# language json, and optionally an dict of values that could come from
# godot memory, formatted as { compID : { keys and values to add} ...}
static func UnitFromJSONS(structure: String, langStruct: String, langISO: String, data: Dictionary = {}) -> Dictionary:
	var structDict: Dictionary = JSON.parse_string(structure)
	var langDict: Dictionary = JSON.parse_string(langStruct)
	
	return _BuildUnitActivation(structDict, langDict, langISO, data)

# Build Unit Activation Dict
static func _BuildUnitActivation(struct: Dictionary, lang: Dictionary,
	langISO: String, data: Dictionary) -> Dictionary:
	
	var unitAct := {}
	
	# Build unit activation minus the components array
	for key in struct.keys():
		if key == "components": continue
		unitAct[key] = struct[key]
	
	var outputComponents := []
	
	for givenComponent_struct in struct["components"]:
		
		var toAppend := {}
		
		# Get prerequisite data
		# Get Language dict if available
		var givenComponent_lang := {}
		if givenComponent_struct["ID"] in lang.keys():
			givenComponent_lang = lang[givenComponent_struct["ID"]]
		# get data dict if available
		var givenComponent_data := {}
		if givenComponent_struct["ID"] in data.keys():
			givenComponent_data = data[givenComponent_struct["ID"]]
		
		if (givenComponent_struct["type"] == "tab") or (givenComponent_struct["type"] == "box") or (givenComponent_struct["type"] == "popup") or (givenComponent_struct["type"] == "collapsible"):
			
			# We are dealing with a subunit, time for recursion
			# This is not particuarly efficient. Too Bad!
			toAppend = _BuildUnitActivation(givenComponent_struct, givenComponent_lang, 
				langISO, givenComponent_data)
		
		else:
			# add in component struct data
			toAppend = givenComponent_struct.duplicate()
			
			# add in component lang data
			for langKey in givenComponent_lang.keys():
				
				var possibleLangsForOutput: Dictionary = givenComponent_lang[langKey]
				var langInput
				if langISO not in possibleLangsForOutput.keys():
					langInput = possibleLangsForOutput[FALLBACK_LANG]
				else:
					langInput = possibleLangsForOutput[langISO]
				
				toAppend[langKey] = langInput
			
			# add in component data data
			toAppend.merge(givenComponent_data, true)
		
		# completed our dict to append. Append and move on
		outputComponents.append(toAppend)

	unitAct["components"] = outputComponents
	return unitAct

# Read txt / json file
static func ReadTextFile(path: String) -> String:
	var file = FileAccess.open(path, FileAccess.READ)
	if file == null: return ""
	var text = file.get_as_text()
	file.close()
	return text

# Given the UnitID, the language ISO code, and additonal data, generate a Unit Activation
static func GenerateDefinedUnitDict(unitID: String, langISO: String,
	additionalData: Dictionary = {}) -> Dictionary:
	
	var structPath: String = PATH_UISTRUCTS + unitID + ".JSON"
	var langPath: String = PATH_LANGUAGES + unitID + "_L.JSON"
	
	
	var structStr = ReadTextFile(structPath)
	var langStr = ReadTextFile(langPath)
	
	if langStr == "": langStr = "{}"
	
	return UnitFromJSONS(structStr, langStr, langISO, additionalData)

# Checks if the first vector2 is smaller than the second in any dimension
static func IsVector2SmallerInAnyDim(isSmallerV2: Vector2, isLargerV2: Vector2) -> bool:
	if isSmallerV2.x < isLargerV2.x: return true
	if isSmallerV2.y < isLargerV2.y: return true
	return false

# Clamps the requested Vector2 to the largest allowed size per dimension as per the limit
static func ClampVector2ToLargestAllowed(requested: Vector2, limit: Vector2) -> Vector2:
	if requested.x > limit.x: requested.x = limit.x
	if requested.y > limit.y: requested.y = limit.y
	return requested

# Expands the requested Vector2 to the smallest allowed size per dimension as per the limit
static func GrowVector2ToSmallestAllowed(requested: Vector2, limit: Vector2) -> Vector2:
	if requested.x < limit.x: requested.x = limit.x
	if requested.y < limit.y: requested.y = limit.y
	return requested

static func MinimumDistanceBetween2Sizes(a: Vector2, b: Vector2) -> Vector2:
	return (a + b) / 2.0 # yes, my function name is longer than the function itself

# Returns float array from Vector2 array, either from X or Y
static func GetSingleDimensionFromVector2Arr(input: Array, isX: bool) -> Array:
	var output: Array = []
	for e in input:
		if isX: output.append(e.x)
		else: output.append(e.y)
	return output

# Sums entire float array
static func SumFloatArray(input: Array) -> float:
	var output := 0.0
	for e in input:
		output = output + e
	return output

# Returns the sum of an array from the start to a defined end
static func SumFloatArrayAtIndex(arr: Array, stopIndex: int) -> float:
	var total := 0.0
	for i in range(stopIndex):
		total += arr[i]
	return total

# Returns Vector in order of available components, starting with default, then
# full vector, then component variables
static func LoadMostDefaultV2(dict: Dictionary, keyName: String, defaultValue: Vector2) -> Vector2:
	var output: Vector2 = GetIfCan(dict, keyName, defaultValue)
	output.x = GetIfCan(dict, keyName + "X", output.x)
	output.y = GetIfCan(dict, keyName + "Y", output.y)
	return output

static func LoadMostDefaultv3_Color(dict: Dictionary, keyName: String, defaultValue: Vector3) -> Vector3:
	var output: Vector3 = GetIfCan(dict, keyName, defaultValue)
	output.x = GetIfCan(dict, keyName + "R", output.x)
	output.y = GetIfCan(dict, keyName + "G", output.y)
	output.z = GetIfCan(dict, keyName + "B", output.z)
	return output

static func V3IToColor(input: Vector3i) -> Color:
	var output = Color(0,0,0)
	output.r8 = input.x
	output.g8 = input.y
	output.b8 = input.z
	output.a8 = 255
	return output

static func ColorToV3I(input: Color) -> Vector3i:
	var output = Vector3i(0,0,0)
	output.x = input.r8
	output.y = input.g8
	output.z = input.b8
	return output

static func clampToRange(input: float, min: float, max: float) -> float:
	if input < min: return min
	if input > max: return max
	return input

# Remaps X Y floats from activation JSON into a vector
static func RemapVector2FloatsToVector2(prefix: String, searching: Dictionary) -> Dictionary:
	if (prefix in searching.keys()): return searching # vector present
	var X: float = GetIfCan(searching, prefix + "X", 0.0)
	var Y: float = GetIfCan(searching, prefix + "Y", 0.0)
	
	var vec = Vector2(X, Y)
	if vec == Vector2(0.0, 0.0): return searching # value wasnt present
	
	searching[prefix] = vec
	return searching

# Remaps X Y Z floats from activation JSON into a vector
static func RemapVector3FloatsToVector3(prefix: String, searching: Dictionary) -> Dictionary:
	if (prefix in searching.keys()): return searching # Vector present
	var X: float = GetIfCan(searching, prefix + "X", 0.0)
	var Y: float = GetIfCan(searching, prefix + "Y", 0.0)
	var Z: float = GetIfCan(searching, prefix + "Z", 0.0)
	
	var vec = Vector3(X, Y, Z)
	if vec == Vector3(0.0, 0.0, 0.0): return searching # value wasnt present
	
	searching[prefix] = vec
	return searching

# Remaps R G B floats from activation JSON into a vectori
static func RemapRGBToVector3i(prefix: String, searching: Dictionary) -> Dictionary:
	if (prefix in searching.keys()): return searching
	var X: int = GetIfCan(searching, prefix + "R", 0)
	var Y: int = GetIfCan(searching, prefix + "G", 0)
	var Z: int = GetIfCan(searching, prefix + "B", 0)
	
	var vec = Vector3i(X, Y, Z)
	if vec == Vector3i(0, 0, 0): return searching # value wasnt present
	return searching

static func CheckIfSubkeyExists(dictIn: Dictionary, searchText: String) -> bool:
	for key in dictIn.keys():
		if key.contains(searchText): return true
	return false
