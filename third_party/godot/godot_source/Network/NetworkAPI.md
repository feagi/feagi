## Network API for GET & POST

Handles Get / Post requests on separate threads using individual workers to avoid execution pauses when running the scene.


Use the SimpleNetworkAPI scene / node:
-	add the SimpleNetworkAPI.tscn to the scene
-	On it, run Call ( request, method, functionToRecieveData )
	-	request: (str) the full URL to send the request to
	-	method: The method to use
		-	HTTPClient.METHOD_POST
		-	HTTPClient.METHOD_GET
	-	functionToRecieveData: The function you wish to be activated once the data is received and processed. This is done via a signal
		-	*This is likely not to occur for a certain length of time, beware of execution order, do not have code executing within the same frame of the call depend on the output!*
		-	Function of choice must accept (in order):
			-	result (int)
			-	response_code (int)
			-	headers (PackedStringArray)
			-	body (PackedByteArray)
			-	More info: https://docs.godotengine.org/en/stable/classes/class_httprequest.html (under signals)

- To change headers used, change the 'headersToUse' array variable 
- minWorkersAlive can be changed to scale the number of available workers at any time


