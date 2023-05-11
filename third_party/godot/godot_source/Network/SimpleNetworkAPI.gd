extends Node
class_name SimpleNetworkAPI

# Default Config
const DEF_MINWORKERSALIVE = 5
const DEF_HEADERSTOUSE = ["Content-Type: application/json"]

var minWorkersAlive: int = DEF_MINWORKERSALIVE
var immediateWorkersAvailable: Array
var headersToUse: Array:
	get: return _headersToUse
	set(v) : _UpdateHeaders(v)
var numWorkersAvailable: int:
	get: return immediateWorkersAvailable.size()

var _workerScene: PackedScene = preload("res://Network/SimpleNetworkWorker.tscn")
var _headersToUse: Array = DEF_HEADERSTOUSE

## Used to make a network call
func Call(request: String, method: int, functionToRecieveData, dataIn = {}) -> void:
	var worker: SimpleNetworkWorker
	if numWorkersAvailable > 0:
		worker = immediateWorkersAvailable.pop_back()
	else: 
		worker = _SpawnWorker()
	worker.Call(request, method, functionToRecieveData, dataIn)


func _init() -> void:
	for i in range(minWorkersAlive):
		_SpawnWorker(true)


# Handles Spawning Worker Logic
func _SpawnWorker(addToAvailable: bool = false) -> SimpleNetworkWorker:
	var worker: SimpleNetworkWorker = _workerScene.instantiate()
	worker.API = self
	worker.headers = headersToUse
	if addToAvailable:
		immediateWorkersAvailable.append(worker)
	add_child(worker)
	return worker

# updates the headers of all inactive workers
func _UpdateHeaders(newHeaders: Array) -> void:
	for worker in immediateWorkersAvailable:
		worker.headers = newHeaders
