extends Element_Base
class_name Element_Vector3

const D_editable = true
const D_vectorValue = Vector3(0,0,0)
const D_subLabels = ["X", "Y", "Z"]

const _specificSettableProps = {
	"vectorValue": TYPE_VECTOR3,
	"editable": TYPE_BOOL,
	"subLabels": TYPE_ARRAY
}

var vectorValue: Vector3:
	get: return Vector3(_LineEdits[0].value, _LineEdits[1].value, _LineEdits[2].value)
	set(v):
		_LineEdits[0].value = v.x
		_LineEdits[1].value = v.y
		_LineEdits[2].value = v.z

var editable: bool:
	get: return _LineEdits[0].editable
	set(v): 
		_LineEdits[0].editable = v
		_LineEdits[1].editable = v
		_LineEdits[2].editable = v

var subLabels: Array:
	get:
		return [_LineEdits[0].text, _LineEdits[1].text, _LineEdits[2].text]
	set(v):
		_Labels[0].text = v[0]
		_Labels[1].text = v[1]
		_Labels[2].text = v[2]

var _LineEdits: Array
var _Labels: Array
var _Conts: Array


func _ActivationSecondary(settings: Dictionary) -> void:
	_Conts = get_children()
	if(_has_label): _Conts.remove_at(0)
	
	for i in range(3):
		_Labels.append(_Conts[i].get_child(0))
		_LineEdits.append(_Conts[i].get_child(1))
		_LineEdits[i].value_edited.connect(_DataUpProxy)
	
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	vectorValue = HelperFuncs.GetIfCan(settings, "vectorValue", D_vectorValue)
	subLabels = HelperFuncs.GetIfCan(settings, "subLabels", D_subLabels)
	
	
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["vector3"]

func _getChildData() -> Dictionary:
	return {
		"value": vectorValue,
	}

func _DataUpProxy(_data) -> void: # The data from a single FF is irrelevant, send the whole vector
	DataUp.emit({"value": vectorValue}, ID, self)
