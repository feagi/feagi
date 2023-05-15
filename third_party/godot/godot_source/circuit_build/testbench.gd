extends Node




var loadDict: Dictionary = {
	"type" : "field",
	"label" : "Field 1",
	"fieldWidth" : 300,
}

var loadDict2: Dictionary = {
	"type" : "counter",
	"label" : "Field 2",
}

var loadDict3: Dictionary = {
	"type" : "dropdown",
	"label" : "dd",
	"options" : ["a", "b", "c"],
	"hasButton" : true
}

var loadDict4: Dictionary = {
	"type" : "header",
	"label" : "wordswordswords",
}

var complist = [loadDict, loadDict2, loadDict3, loadDict4]
var _UnitScene: PackedScene = preload("res://UI/Units/unit.tscn")
var NetworkAPI: SimpleNetworkAPI

func testprint(result, response_code, headers, body):
	var teststr: String = body.get_string_from_utf8()
	print(teststr)
	#complist[2]["options"] = JSON.parse_string(teststr)
	
	var newUnit = _UnitScene.instantiate()
	add_child(newUnit)
	newUnit.Activate(complist, Vector2(500,244))
	
	

func _ready():
	NetworkAPI = SimpleNetworkAPI.new()
	add_child(NetworkAPI)
	NetworkAPI.Call("http://192.168.50.246:8000/v1/feagi/genome/cortical_area_id_list", HTTPClient.METHOD_GET, testprint )
	
