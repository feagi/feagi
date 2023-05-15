
### **DropDown**
Used to show multiple options. Optional button to trigger action

- *label*: (str) The label / description preceding the field
	-  Can be read/set from activation dict or as property
- *hasButton*: (bool) Has a button to the right for adding or some other purpose. Defaults to False.
	- Can be set from activation dict, then only read as a property
- *buttonText*: (str) Text inside of the button. Defaults to "+"
	- Can be read/set from activation dict or as property
	- Doesn't do anything if the button isn't enabled
- *options*: (str array) Text options within dropDown. REQUIRED. 
	- Can be read/set as a property, but MUST be set from activation
	- Depending on changes, may force changes for index / value if previously selected option becomes unavailable
- *value*: (str) current dropdown setting. 
	- Can be read/set from activation dict or as property
- *initIndex*: (int) default index to initialize to. Defaults to -1 (none).
	- Can't only be set from activation dict. Not readable nor settable as a property

Outputs into the Data signal the following dict on selection change:
"selected" : [str selection]

Outputs into the Data signal the following dict on button press change:
"button" : true