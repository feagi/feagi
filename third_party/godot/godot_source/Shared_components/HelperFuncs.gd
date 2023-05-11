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
	var unitID: String = structDict["ID"]
	var langDict: Dictionary = JSON.parse_string(langStruct)
	
	var outputDict: Dictionary = {"ID": unitID}
	var newComponents: Array = []
	
	
	for compDict in structDict["components"]:
		# Specific Component ID we are on in the Component Array
		var compID: String = compDict["ID"]
		var finalCompDict: Dictionary = compDict.duplicate(true)
		
		# Check if language Dict has anything to add
		if compID in langDict.keys():
			# Merge in language dict
			var dictToMerge: Dictionary = {}
			for compInLangDict in langDict[compID].keys():
				if langISO in langDict[compID][compInLangDict].keys():
					dictToMerge.merge({compInLangDict: langDict[compID][compInLangDict][langISO]})
				else:
					# fall back to backup language
					dictToMerge.merge({compInLangDict: langDict[compID][compInLangDict][FALLBACK_LANG]})
			
			finalCompDict.merge(dictToMerge)
		
		# Check if data array has anything to add
		if compID in data.keys():
			# Merge in data dict
			finalCompDict.merge(data[compID])
		
		# Component is now complete, append to array
		newComponents.append(finalCompDict)
	# end
	outputDict["components"] = newComponents
	
	# append additional Unit Properties
	if "isVertical" in structDict.keys():
		outputDict["isVertical"] = structDict["isVertical"]
	
	return outputDict

# Read txt / json file
static func ReadTextFile(path: String) -> String:
	var file = FileAccess.open(path, FileAccess.READ)
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


