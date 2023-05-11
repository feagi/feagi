### **Counter**
Used for user defined input of numbers. 

 - *label*: (str) The label / description preceding the field
	 - Can be read/set from activation dict or as property
 - *editable*: (bool) Allow editing of this field. Default true
	 - Can be read/set from activation dict or as property
 - *counterWidth*: (float) How wide the counter itself should be. Default 100
	 - Can be read/set from activation dict or as property
 - *isInt*: (bool) Round all inputs to ints. Default False
	 - Can be read/set from activation dict or as property
 - *prefix*: (str) Prefix to show before number. Default ""
	 - Can be read/set from activation dict or as property
 - *suffix*: (str) Suffix to show after number. Default ""
	 - Can be read/set from activation dict or as property
 - *maxValue*: (float) Maximum Accepted Value. Default 1000
	 - Can be read/set from activation dict or as property
 - *minValue*: (float) Minimum Accepted Value. Default -1000
	 - Can be read/set from activation dict or as property
 - *interval*: (float) Step used by the arrows. Default 1
	 - Can be read/set from activation dict or as property
 - *value*: (int or float) Actual value inside of field
	 - Can be read/set from activation dict or as property
	 - Signals up when changed

Outputs into the Data signal the following dict on value change:
"number" : [value]