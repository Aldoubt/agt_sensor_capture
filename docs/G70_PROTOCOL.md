# WHEELTEC G70 Protocol

v0.1 uses UBX NAV-PVT as the primary acquisition protocol.

## One-time receiver setup

Configure the receiver in u-center before field use:

- UBX output enabled
- UBX-NAV-PVT enabled
- navigation output rate 5 Hz
- serial baud initially 9600 unless field validation requires a higher rate
- save the receiver configuration

The runtime ROS driver is read-only. It does not configure rate, dynamic model, GNSS constellations, TMODE/Survey-In, message rates or receiver flash.

RAWX/SFRBX are not required for v0.1 trajectory truth and are deferred to future raw-observation/PPK work.
