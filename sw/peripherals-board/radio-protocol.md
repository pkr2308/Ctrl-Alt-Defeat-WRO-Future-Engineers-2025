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

#### Block 0
|Type|Name|Description|Size|
|----|----|-----------|----|
|uint8_t|Block ID|Describes what block to parse data as|8 bits|