extends HTTPRequest
class_name SimpleNetworkWorker

var API: SimpleNetworkAPI
var headers: Array


func Call(requestAddress: String, method: int, functionToRecieveData, dataIn) -> void:
	request_completed.connect(functionToRecieveData) # reverse signal
	match(method):
		HTTPClient.METHOD_GET:
			request(requestAddress, headers, method)
			return
		HTTPClient.METHOD_POST:
			request(requestAddress, headers, method, JSON.stringify(dataIn))
			return
		HTTPClient.METHOD_PUT:
			request(requestAddress, headers, method, JSON.stringify(dataIn))
			return
		HTTPClient.METHOD_DELETE:
			request(requestAddress, headers, method, JSON.stringify(dataIn))
			return
	@warning_ignore("assert_always_false")
	assert(false, "Invalid HTTP request type")


func _init() -> void:
	self.use_threads = false # enable simple multithreading


func request_completed_t(_result, _response_code, _headers, _body) -> void:
	_QueryForDestruction()


# Checks if there is space in the API available array and if so, adds self there.
# Otherwise, self destructs
func _QueryForDestruction() -> void:
	if API.NumWorkersAvailable < API.MinWorkersAlive:
		API.ImmediateWorkersAvailable.append(self)
	else:
		self.queue_free() # Self Destruct
