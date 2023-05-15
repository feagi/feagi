### **Toggle**
Used to show boolean options
- *label*: (str) The label / description preceding the field
	-  Can be read/set from activation dict or as property
- *isCheckBox*: (bool) Renders as a Checkbox. Otherwise renders as a Toggle Switch. Default False.
	- Can only be set from activation dict, cannot be changed as a property after init. Can be get as a property
- *value*: (bool) Current state of toggle
	- Can be read/set from activation dict or as property
- *editable*: (bool) Can user interact with this toggle. Default True
	-  Can be read/set from activation dict or as property

Outputs into the Data signal the following dict on boolean change:
"state" : [bool]
