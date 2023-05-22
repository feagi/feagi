


# Units
A unit in this context is the name for a "node" you canuse as a panel for UI in general. Called "Unit" to avoid confusion with Godot nodes.

Units have some properties that are set to the unit itself, such as its padding. But they also contain *components*, which is the main way to add parts to a Unit. *Note that Units will refuse to become narrower than the width/height of their largest component*. It is possible to use Units as components for other Units.

**properties**

 - ID: (str)
	 - set at activation only, **required**
	 - returns the string ID name of the Unit, used with the JSON init and with passing data through
 - isHorizontal: (bool)
	 - set at activation only
	 - returns whether the internal components are laid horizontally (true) or vertically (false). Default is True
 - dataSignalAvailable: (bool)
	 - returns whether this Unit has an accessible data signal
 - componentIDs: (str Array)
	 - returns the component IDs that are within this Unit
 - componentRefs: (Dict)
	 - returns a dictionary of Component References, key'd by their IDs
 - isSubUnit: (bool)
	 - returns true if the Unit is a subunit.
	 -	minimumSize: (Vector2)
	 -	returns: The cached minimum dimensions that the internal nodes allow. *To force update this, call "_UpdateMinimumDimensions"!*
 - padding: (Vector2) 
	 - padding on sides of the Unit
	 - set at activation, and can be updated as a property
		 - setting triggers a minimum size recalculation, and if necessary, Size change
	 - returns the padding on the sides of the Unit (half on either side). Default is <6.0, 3.0>
 - Hsize: (Vector2)
	 - can be set at activation or as a property
		 - Unit will refuse to shrink down smaller than minimum allowed size
		 - Will signal up a SizeChanged signal
	 - returns the size of the Unit back panel
 - componentsSpawnPoint: (Vector2)
	 - Returns the relative spawn position of components
	 - Can only be set at activation
- componentData: (Dict)
	- Can only be set / get at runtime
	- returns the data of all components in a dictionary, key'd by their ID
	- can be set to overwrite values in internal components, in this fashion:
		- {ComponentID : {Variable/property name: data}}
- enableTitleBar: (bool)
	- Can only be set at activation, defaults to false
	- enables a title bar at the top of the Unit
- enableCloseButton: (bool)
	- Can only be set at activation and requires 'enableTitleBar' to work, defaults to false
	- enables a close button on the top left of the Unit
- titleBarTitle: (string)
	- Can only be set at activation and requires 'enableTitleBar' to work, defaults to 'UNNAMED TITLE'
	- defines the title on the top of the Unit


## Activation Dictionary Notes
Units on initialization are blank, and are activated with their data and connections with "Activation" Dictionaries. They can be generated at runtime, but it may be easier to use statically created Units instead with JSONs (and add additional data during runtime, as explained later).
To create Unit JSONs, place the files in the following locations:
 - /UI_Structures/(UNIT_ID).JSON -> structure file
 - /Language/(UNIT_ID)_L.JSON ->  language file

Structure files are formatted as such:
{
	"ID": (str ID of the unit),
	(Other Properties): (Data that goes with them, see above),
	"components": [
		{"ID": (first component ID),
		 "type": (the type of component to spawn, see available components)
		 (other properties): (technically you can set whatever here, but keep language / text for the language file. Some components have required additional properties, so read their documentations)
		 },
		 {"ID": (second component ID)... } 
	]
}

Language files should mainly hold language text, and all component texts are matched by component ID
{
	(ID of component to match with): {
		(property to write to): { (language ISO code) : ( data to put in )}
	}
	(second ID of component to match with...)
}
if a non English language is selected for neurorobotics studio, but a component is missing that language for a specific bit of text, then that specific text will fall back to English.

Once all that is defined, you can generate the Unit (preferably from the UI_Manager) with the function:
HelperFuncs.GenerateDefinedUnitDict( (str ID of the unit), (str current language ISO code), (OPTIONAL extra data)).

The extra data is a dictionary of data you can add/overwrite the json activations with, that can be generated at runtime. The expected format is:
{
	(ComponentID): {
		(property to overwrite / add): data
	}
}