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

#### Block 0 (total size: 24 bytes, sent on pipe 0)
|Type|Name|Description|Size|
|----|----|-----------|----|
|float|Orientation X|X-axis vehicle orientation|4 bytes|
|float|Orientation Y|Y-axis vehicle orientation|4 bytes|
|float|Orientation Z|Z-axis vehicle orientation|4 bytes|
|float|Left LiDAR distance|Distance measured from left LiDAR in centimeters|4 bytes|
|float|Front LiDAR distance|Distance measured from front LiDAR in centimeters|4 bytes|
|float|Right LiDAR distance|Distance measured from right LiDAR in centimeters|4 bytes|

#### Block 1 (total size: 24 bytes, sent on pipe 1)
|Type|Name|Description|Size|
|----|----|-----------|----|
|float|Acceleration X|X-axis vehicle acceleration|4 bytes|
|float|Acceleration Y|Y-axis vehicle acceleration|4 bytes|
|float|Acceleration Z|Z-axis vehicle acceleration|4 bytes|
|float|Gyroscope Z|Z-axis vehicle angular rates|4 bytes|
|float|Gyroscope Z|Z-axis vehicle angular rates|4 bytes|
|float|Gyroscope Z|Z-axis vehicle angular rates|4 bytes|