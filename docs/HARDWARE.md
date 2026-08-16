# Hardware Assumptions

## MID360 and camera

- Livox MID360
- Hikrobot industrial camera supported by the installed MVS SDK
- camera external trigger source: `Line0`
- trigger activation: `FallingEdge`
- target camera rate: 10 Hz
- fixed exposure target: 5000 us for the initial baseline

The hardware trigger path must be verified on the real platform before acceptance testing.

## G70

- WHEELTEC G70 single-antenna RTK receiver
- Linux device alias: `/dev/wheeltec_gnss`
- UBX NAV-PVT at 5 Hz
- G70 PPS is not connected into the common timing architecture in v0.1
