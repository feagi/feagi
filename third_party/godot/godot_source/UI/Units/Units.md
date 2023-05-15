

# Units
A unit in this context is the name for a "node" you can put down either in a node graph or use as a panel for UI in general. Called "Unit" to avoid confusion with Godot nodes.

Units have some properties that are set to the unit itself, such as if its draggable. But they also contain *components*, which is the main way to add parts to a Unit. *Note that Units will refuse to become narrower than the width/height of their largest component*

**Properties**
 - unitID: (str) The ID of the Unit
	 - MUST be set from the activation dict, and then can be read
 - isVertical: (bool) Should Components stack vertically? Default True
	 - Can only be set from Activation dict, and then read as property
 - padding: (Vector2) Padding distances on edges of Unit
	 - Can be read/set from activation dict or as property
		 - setting causes a resize event
- position: (Vector2) Position of Unit on screen, from the upper left corner. Defaults to (0,0)
	- Can be read/set from activation dict or as property
- size: TODO - don't use
- componentsSpawnPoint: (Vector2) Where components spawn in
	- Can only be read as a property
- minimumSize: (Vector2) The minimum allowed size of the Unit
	- Can only be read as a property
-  componentData: (Dictionary) Returns a dict consisting of keys of each component capable of having a value (IE not headers) ID, and for each stores the value of that component
	- Can only be read, not set by any direct means
- componentIDs: (str[]) Array of IDs of composing components
	- Can only be read, not set by any direct means
- componentRefs: (dict) Returns the components by references, each referenced by a key of their compID
	- Can only be read, not set by any direct means


**Methods**

- AddComponent ( component, checkResizing) -> void
	- Adds components to the Unit at the end
	- Inputs:
		- component: Dictionary - component activation dictionary that defines the type of component and its settings
		- checkResizing: bool - OPTIONAL, defaults true. Enables checking for minimum sizing changes of the Unit

- AddMultipleComponents( components, AdjustSizeAfter) -> void
	- Adds an array of components in order, using the 'AddComponent' function repeatedly
	- Inputs:
		- components: Array - Array of activation dictionaries
		- AdjustSizeAfter: bool - OPTIONAL, defaults true, enables checking for minimum sizing changes after all components have been added
- ApplyMinimumSize( force ) -> void
	- Checks if the minimum size allowed changes, and rescales the Unit if so (and signals upwards if relevant #TODO)
		- You generally do not need to call this since it should be called automatically during normal resize calling events
	- Inputs:
		- force: bool - OPTIONAL, defaults false, forces a resize event even if the minimum size is unchanged
- RelayInputDataToComps( input ) -> void
	- Instead of setting properties of components directly, this can be used to apply property changes to components of the Unit.
		- Some fail safes do exist in this to prevent invalid inputs, be sure to check the log!
	- Inputs:
		- input: Dictionary - input dictionary with properties to change for desired components
		- Format:
			- { CompID: { propertyName: new value} }
			- you can have multiple CompIDs and Propertynames