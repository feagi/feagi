### **Header**
Used to define a header to define sections in a panel. Purely Visual
- *label*: (str) The text of the header
	- Can be read/set from activation dict or as property
- *side*: (int) The side that the Header will be on. Defaults to 1
	- 0 is left, 1 is center, 2 is right
	- Can only be set from activation dict, cannot be changed as a property after init. Can be get as a property