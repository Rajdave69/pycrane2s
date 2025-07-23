# BLE Characteristics -- These were found using the nRF Connect App
# There are only two

# Known as write-no-response. All commands are sent to this
WRITE_UUID = "d44bc439-abfd-45a2-b575-925416129600"

# Everything sent by the gimbal will be found here, including heartbeats.
NOTIFY_UUID = "d44bc439-abfd-45a2-b575-925416129601"