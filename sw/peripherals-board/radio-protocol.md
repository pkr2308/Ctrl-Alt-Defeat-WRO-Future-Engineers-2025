## CTRL+ALT+DEFEAT Peripherals Board Radio Communication Protocol

### Pipes
#### Vehicle
|Pipe #|Address|Purpose|
|------|-------|-------|
|0|CARRX|Receives commands from TLM as acknowledgement payload on this pipe|
|1|Undefined|Unused|
|2|Undefined|Unused|
|3|Undefined|Unused|
|4|Undefined|Unused|
|5|Undefined|Unused|
#### Telemetry Adapter
|Pipe #|Address|Purpose|
|------|-------|-------|
|0|TLM00|Telemetry module receives first block of telemetry on this pipe|
|1|Undefined|Unused|
|2|Undefined|Unused|
|3|Undefined|Unused|
|4|Undefined|Unused|
|5|Undefined|Unused|


### Telemetry blocks

#### Block 0 (total size: 25 bytes)
|Type|Name|Description|Size|
|----|----|-----------|----|
|uint8_t|Block ID|Describes what block to parse data as|1 byte|
|float|Orientation X|X-axis vehicle orientation|4 bytes|
|float|Orientation Y|Y-axis vehicle orientation|4 bytes|
|float|Orientation Z|Z-axis vehicle orientation|4 bytes|
|float|Left LiDAR distance|Distance measured from left LiDAR in centimeters|4 bytes|
|float|Front LiDAR distance|Distance measured from front LiDAR in centimeters|4 bytes|
|float|Right LiDAR distance|Distance measured from right LiDAR in centimeters|4 bytes|