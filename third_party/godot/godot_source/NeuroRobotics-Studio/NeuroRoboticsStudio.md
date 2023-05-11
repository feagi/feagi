# NeuroRobotics Studio
This documentation gives the general layout of the system

```mermaid
graph TD
A[Core] --> B[Global Network System]
A --> D[Global UI System]
A --> C[Feagi Cache]
D --> E(Graph Core)
D --> F(Brain Visualizer)
D --> G(Other UI elements)
```

### Core:
The central manager of the Studio. Receives global input events from below components, directs network calls and data caching events, and pushes out changes to the UI and network in response.

```mermaid
graph LR
A((UI event in))
B((Network event in))
C((Cache event in))
D[Func: RetrieveEvents]
E(Network Call to Global Network System)
F(UIManager function: RelayDownwards)
G(Update Cache)
H(Specific Network Relay Function)
A -- data dict --> D
D -- if network response --> E
D -- if UI response --> F
B --> H
H -- if UI response --> F
C -- if UI response --> F
A --> G
B --> G
```

### Global Network System:
A managed set of workers that handles API calls. Since network calls are non-instant, workers are dynamically created as needed, and will signal out to Core once call is completed

```mermaid
graph LR
A((Network Request))
B[Func: Call]
C(Create Worker)
D((Worker Request Completion))
E(Signal data to given Core Relay Function)
F(Return to idle worker pool)
G(Kill Self)
A -- Request Worker --> B
B -- Not Enough Workers --> C
C --> B
D --> E
E -- Worker Idle Cap not Reached --> F
E -- Enough Idle Workers --> G
```

### Cache:
Holds variables retrieved from Network and other places, as well as calculating and updating compound variables that require several network calls to be generated

```mermaid
graph LR
A((Core reports updated variable))
B(Update stored Cached Variable)
C(Attempt update of compound variable)
D(Keep Waiting)
E(Recalculate Recompound variable)
F(Signal Core)
A --> B
B -- if var is part of compound variable --> C
C -- if not all prereq vars retrieved --> D
C -- All required vars acquirred --> E
E --> F
```

### UI Manager
Parent to all UI elements, relay station for UI Events. Handles creation and management of Units 

```mermaid
graph LR

A((Call from Core))
B((User Input))
C[Func: RelayDownwards]
D[Signal: DataUp]
E(Take action on relevant Unit)
A --> C
C --> E
B -- if state of Studio or Feagi is impacted --> D
B -- if immediate UI change is result --> E
```