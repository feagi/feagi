
### **Button**
It's a button. What do you expect?
- *label*: (str) The text within the button
	-  Can be read/set from activation dict or as property
- *editable*: (bool) Can user interact with this button? Default True
	-  Can be read/set from activation dict or as property

Outputs into the Data signal of only the default type and ID when pressed, nothing else. Treat the arrival of the signal itself as the data you need
