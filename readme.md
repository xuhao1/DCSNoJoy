# Intro

![](doc/demo0.png)

## Usage

Delete all mouse view axis. 
Install prerequest python packages, including 
```
transformations
PyQt5
keyboard
mouse
PyYaml
```

Install DCSEasyControlExports folder to your "Saved Games/DCS/Scripts" Path and add this line to file __Saved Games/DCS/Scripts/Export.lua__

```lua
dofile(lfs.writedir()..[[Scripts\DCSEasyControlExports\DCSEasyControlExport.lua]])
```

Then, run
>python main.py

in powershell.

Set DCS to F12 view.

Modified Configs/config.py for mouse speed and other user experience related parameters. 
Modified controller parameters in Configs/aircraft-name.yaml. If your aircraft does not exist in that folder, just run DCSEasyControl and enter the aircraft in DCS. The aircraft-name.yaml file will be created at once. Once you modified the parameters for aircraft, please pause or enter the ESC menu in DCS and return to the game again, the up to date parameters will be loaded.

## Implement Details

The reference and cooridnate system for DCS api please see this [doc](./doc/dcs.md).
![](./doc/world_axis.PNG)

## LICENSE
LGPL3

## Known Issue
Aircraft not turn with very large movement between aim target and current direction.

Control Stability.

Mouse can run out of screen


## TODO

Parameter system for different aircrafts.
