from collections.abc import Iterable as _Iterable
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper

DESCRIPTOR: _descriptor.FileDescriptor

class CommandType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CMD_UNKNOWN: _ClassVar[CommandType]
    CMD_SET_TARGET: _ClassVar[CommandType]
    CMD_SET_BANDWIDTH: _ClassVar[CommandType]
    CMD_CONTROL_HEATER: _ClassVar[CommandType]
    CMD_TRIGGER_SAR: _ClassVar[CommandType]
    CMD_TRIGGER_CAMERA: _ClassVar[CommandType]
    CMD_JOG_SERVO: _ClassVar[CommandType]
CMD_UNKNOWN: CommandType
CMD_SET_TARGET: CommandType
CMD_SET_BANDWIDTH: CommandType
CMD_CONTROL_HEATER: CommandType
CMD_TRIGGER_SAR: CommandType
CMD_TRIGGER_CAMERA: CommandType
CMD_JOG_SERVO: CommandType

class SetTargetPayload(_message.Message):
    __slots__ = ("target_heading",)
    TARGET_HEADING_FIELD_NUMBER: _ClassVar[int]
    target_heading: float
    def __init__(self, target_heading: float | None = ...) -> None: ...

class SetBandwidthPayload(_message.Message):
    __slots__ = ("rate_kbps",)
    RATE_KBPS_FIELD_NUMBER: _ClassVar[int]
    rate_kbps: int
    def __init__(self, rate_kbps: int | None = ...) -> None: ...

class ControlHeaterPayload(_message.Message):
    __slots__ = ("heater_id", "enable")
    HEATER_ID_FIELD_NUMBER: _ClassVar[int]
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    heater_id: int
    enable: bool
    def __init__(self, heater_id: int | None = ..., enable: bool | None = ...) -> None: ...

class TriggerCameraPayload(_message.Message):
    __slots__ = ("count", "spacing_ms")
    COUNT_FIELD_NUMBER: _ClassVar[int]
    SPACING_MS_FIELD_NUMBER: _ClassVar[int]
    count: int
    spacing_ms: float
    def __init__(self, count: int | None = ..., spacing_ms: float | None = ...) -> None: ...

class JogServoPayload(_message.Message):
    __slots__ = ("servo_id", "target_tick")
    SERVO_ID_FIELD_NUMBER: _ClassVar[int]
    TARGET_TICK_FIELD_NUMBER: _ClassVar[int]
    servo_id: int
    target_tick: int
    def __init__(self, servo_id: int | None = ..., target_tick: int | None = ...) -> None: ...

class CommandRequest(_message.Message):
    __slots__ = ("command_id", "type", "set_target", "set_bandwidth", "control_heater", "trigger_camera", "jog_servo")
    COMMAND_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    SET_TARGET_FIELD_NUMBER: _ClassVar[int]
    SET_BANDWIDTH_FIELD_NUMBER: _ClassVar[int]
    CONTROL_HEATER_FIELD_NUMBER: _ClassVar[int]
    TRIGGER_CAMERA_FIELD_NUMBER: _ClassVar[int]
    JOG_SERVO_FIELD_NUMBER: _ClassVar[int]
    command_id: str
    type: CommandType
    set_target: SetTargetPayload
    set_bandwidth: SetBandwidthPayload
    control_heater: ControlHeaterPayload
    trigger_camera: TriggerCameraPayload
    jog_servo: JogServoPayload
    def __init__(self, command_id: str | None = ..., type: CommandType | str | None = ..., set_target: SetTargetPayload | _Mapping | None = ..., set_bandwidth: SetBandwidthPayload | _Mapping | None = ..., control_heater: ControlHeaterPayload | _Mapping | None = ..., trigger_camera: TriggerCameraPayload | _Mapping | None = ..., jog_servo: JogServoPayload | _Mapping | None = ...) -> None: ...

class CommandResponse(_message.Message):
    __slots__ = ("command_id", "success", "message")
    COMMAND_ID_FIELD_NUMBER: _ClassVar[int]
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    command_id: str
    success: bool
    message: str
    def __init__(self, command_id: str | None = ..., success: bool | None = ..., message: str | None = ...) -> None: ...

class ServoData(_message.Message):
    __slots__ = ("id", "current_tick", "current_speed", "current_load", "voltage_v", "temperature_c", "online")
    ID_FIELD_NUMBER: _ClassVar[int]
    CURRENT_TICK_FIELD_NUMBER: _ClassVar[int]
    CURRENT_SPEED_FIELD_NUMBER: _ClassVar[int]
    CURRENT_LOAD_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_V_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_C_FIELD_NUMBER: _ClassVar[int]
    ONLINE_FIELD_NUMBER: _ClassVar[int]
    id: int
    current_tick: int
    current_speed: int
    current_load: int
    voltage_v: float
    temperature_c: int
    online: bool
    def __init__(self, id: int | None = ..., current_tick: int | None = ..., current_speed: int | None = ..., current_load: int | None = ..., voltage_v: float | None = ..., temperature_c: int | None = ..., online: bool | None = ...) -> None: ...

class GnssData(_message.Message):
    __slots__ = ("latitude", "longitude", "altitude", "fix_ok")
    LATITUDE_FIELD_NUMBER: _ClassVar[int]
    LONGITUDE_FIELD_NUMBER: _ClassVar[int]
    ALTITUDE_FIELD_NUMBER: _ClassVar[int]
    FIX_OK_FIELD_NUMBER: _ClassVar[int]
    latitude: float
    longitude: float
    altitude: float
    fix_ok: bool
    def __init__(self, latitude: float | None = ..., longitude: float | None = ..., altitude: float | None = ..., fix_ok: bool | None = ...) -> None: ...

class SystemHealth(_message.Message):
    __slots__ = ("cpu_temp_c", "uptime_seconds", "limit_active", "disk_used_percent")
    CPU_TEMP_C_FIELD_NUMBER: _ClassVar[int]
    UPTIME_SECONDS_FIELD_NUMBER: _ClassVar[int]
    LIMIT_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    DISK_USED_PERCENT_FIELD_NUMBER: _ClassVar[int]
    cpu_temp_c: float
    uptime_seconds: int
    limit_active: bool
    disk_used_percent: float
    def __init__(self, cpu_temp_c: float | None = ..., uptime_seconds: int | None = ..., limit_active: bool | None = ..., disk_used_percent: float | None = ...) -> None: ...

class TelemetryFrame(_message.Message):
    __slots__ = ("timestamp", "gondola_heading", "target_heading", "imu_online", "servos", "heater1_active", "heater2_active", "gnss", "health")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    GONDOLA_HEADING_FIELD_NUMBER: _ClassVar[int]
    TARGET_HEADING_FIELD_NUMBER: _ClassVar[int]
    IMU_ONLINE_FIELD_NUMBER: _ClassVar[int]
    SERVOS_FIELD_NUMBER: _ClassVar[int]
    HEATER1_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    HEATER2_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    GNSS_FIELD_NUMBER: _ClassVar[int]
    HEALTH_FIELD_NUMBER: _ClassVar[int]
    timestamp: float
    gondola_heading: float
    target_heading: float
    imu_online: bool
    servos: _containers.RepeatedCompositeFieldContainer[ServoData]
    heater1_active: bool
    heater2_active: bool
    gnss: GnssData
    health: SystemHealth
    def __init__(self, timestamp: float | None = ..., gondola_heading: float | None = ..., target_heading: float | None = ..., imu_online: bool | None = ..., servos: _Iterable[ServoData | _Mapping] | None = ..., heater1_active: bool | None = ..., heater2_active: bool | None = ..., gnss: GnssData | _Mapping | None = ..., health: SystemHealth | _Mapping | None = ...) -> None: ...
