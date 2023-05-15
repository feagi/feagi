### **Field**
Used for user defined input of strings. The following are configs set via the settings dictionary

- *label*: (str) The label / description preceding the field
	-  Can be read/set from activation dict or as property
- *editable*: (bool) Allow editing of this field. Default true
	- Can be read/set from activation dict or as property
- *maxCharacters*: (int) How many characters are allowed. Be sure to validate this on the backend. Default 50
	- Can be read/set from activation dict or as property
- *fieldWidth*: (float) How wide the field itself should be. Default 100.0
	- Can be read/set from activation dict or as property
	- causes resize event when changed
- *placeHolder*: (str) placeholder text inside of field. Default empty ""
	- Can be read/set from activation dict or as property
- *value*: (str) Actual value inside of field
- *onlyTriggerWithEnter*: (bool) If text change events only trigger when user concludes change with Enter key. Default true
	- Can only be set from activation dict, cannot be changed as a property after init. Can be get as a property

Outputs into the Data signal the following dict on text change:
"text" : [str text]