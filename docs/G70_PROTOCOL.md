# WHEELTEC G70 Protocol

v0.1 uses UBX NAV-PVT as the primary acquisition protocol.

## One-time receiver setup

Configure the receiver in u-center before field use:

- UBX output enabled
- UBX-NAV-PVT enabled
- navigation output rate 5 Hz
- serial baud initially 9600 unless field validation requires a higher rate
- save the receiver configuration

The runtime ROS driver is read-only. It does not send receiver configuration packets and does not change rate, dynamic model, GNSS constellations, time mode, message rates or receiver flash.

RAWX/SFRBX are not required for v0.1 trajectory truth and are deferred to future raw-observation/PPK work.

## Runtime parser

`agt_g70_driver` accepts UBX framing (`0xB5 0x62`), bounds payloads to 1024 bytes, validates the UBX Fletcher checksum and decodes only NAV-PVT (`class 0x01`, `id 0x07`, 92-byte payload) into the benchmark-facing data model. Other valid UBX messages are ignored rather than treated as errors.

Retained NAV-PVT evidence includes UTC date/time plus nanoseconds and time accuracy, iTOW, fix type, GNSS-fix and differential flags, carrier solution state, satellite count, position/height, hAcc/vAcc, N/E/D velocity, sAcc, motion heading and heading accuracy.

## Timestamp semantics

Three times remain distinct:

- **GNSS sensor time:** NAV-PVT UTC reconstructed only when date/time validity bits are set and UTC is fully resolved
- **host receive time:** system-clock timestamp captured when the serial read containing the complete frame reaches the ROS process
- **bag record time:** recorder receipt time, owned by rosbag2

For a valid fully-resolved NAV-PVT sample:

```text
NavSatFix.header.stamp          = GNSS sensor UTC
velocity.header.stamp           = GNSS sensor UTC
GnssStatus.header.stamp         = GNSS sensor UTC
TimeReference.time_ref          = GNSS sensor UTC
TimeReference.header.stamp      = host receive time
GnssStatus.host_receive_time    = host receive time
```

If UTC is not usable, standard data messages fall back to host receive time and `GnssStatus.time_valid=false`. No synthetic GNSS time is invented.

## Position and velocity semantics

`NavSatFix.altitude` uses NAV-PVT height above the WGS84 ellipsoid, matching the ROS message definition. MSL height remains available inside the parser model but is not substituted into `NavSatFix.altitude`.

NAV-PVT velocity is NED. The canonical ROS velocity topic uses ENU axes:

```text
linear.x = East
linear.y = North
linear.z = Up = -Down
```

Position covariance uses `hAcc^2, hAcc^2, vAcc^2` on the diagonal when a valid position fix is present. Velocity linear covariance uses `sAcc^2` on the three linear diagonal entries.

## RTK truth filtering

Do not infer RTK fixed/float from `sensor_msgs/NavSatStatus`. The canonical truth-quality field is:

```text
/agt/sensors/gnss/status.carrier_solution
```

Benchmark ground-truth selection should require `CARRIER_FIXED` and a usable GNSS sensor timestamp unless the experiment explicitly defines another policy.
