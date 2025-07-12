import asyncio
import struct
import logging
from bleak import BleakClient

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class Crane2S:
    """
    Low-level Crane 2S Gimbal BLE controller.
    Provides raw packet sending, connection management, and basic motion commands.
    """
    def __init__(
        self,
        address: str,
        write_uuid: str = "d44bc439-abfd-45a2-b575-925416129600",
        notify_uuid: str = "d44bc439-abfd-45a2-b575-925416129601"
    ):
        self.address = address

        # The gimbal has two characteristics, write-no-response and notify
        self.write_uuid = write_uuid
        self.notify_uuid = notify_uuid

        # The data format for sending data to the gimbal includes a byte that increments with each packet sent
        # _seq stores the increment number.
        self._seq = 0
        self._client: BleakClient = None
        self._heartbeat_enabled = False

    # For `async with ...`
    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.disconnect()

    async def connect(self, timeout: float = 10.0):
        logger.info(f"Connecting to {self.address}...")

        # Connect to the given MAC address with bleak
        self._client = BleakClient(self.address)
        await self._client.connect(timeout=timeout)

        if not self._client.is_connected:
            raise ConnectionError(f"Failed to connect to {self.address}")
        if self.notify_uuid:
            await self._client.start_notify(self.notify_uuid, self._on_notify)
            self._heartbeat_enabled = True

        logger.info(f"Connected to {self.address}")

    async def disconnect(self):
        if self._client and self._client.is_connected:
            if self.notify_uuid:
                self._heartbeat_enabled = False
                await self._client.stop_notify(self.notify_uuid)
            await self._client.disconnect()
            logger.info(f"Disconnected from {self.address}")

    async def send_cmd(self, cmd_id: int, value: int, speed: int):
        """
        Send a single motion command (pan/tilt/roll) without repeat.
        """
        pkt = self._build_cmd(cmd_id, value, speed)
        logger.debug(f"Sending packet: {pkt.hex()}")
        await self._client.write_gatt_char(self.write_uuid, pkt, response=False)

    def _build_cmd(self, cmd_id: int, value: int, speed: int) -> bytearray:
        pkt = bytearray(b"\x24\x3C")
        pkt += (8).to_bytes(2, 'little')                # length
        pkt += bytes.fromhex('1812')                    # format ID
        pkt += self._seq.to_bytes(1, 'little')          # sequence
        self._seq = (self._seq + 1) & 0xFF
        pkt += b"\x01"                                # direction
        pkt += cmd_id.to_bytes(1, 'little')             # command
        pkt += (value & 0x0FFF).to_bytes(2, 'little')    # value
        pkt += max(1, min(speed, 255)).to_bytes(1, 'little')  # speed
        crc = self._xmodem_crc16(pkt[4:])               # CRC over bytes 4–11
        pkt += crc.to_bytes(2, 'little')
        return pkt

    # A XMODEM CRC-16 over bytes 4-13 has to be appended at the end of the packet
    def _xmodem_crc16(self, data: bytes) -> int:
        crc = 0x0000
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    async def pan(self, value: int, speed: int, duration: float = 1.0):
        """Continuous pan: 0–2047 right, 2049–4095 left."""
        await self._motion_loop(0x02, value, speed, duration)

    async def tilt(self, value: int, speed: int, duration: float = 1.0):
        """Continuous tilt: 0–2047 up, 2049–4095 down."""
        await self._motion_loop(0x01, value, speed, duration)

    async def pan_pct(self, pct: float, speed_pct: float = 0.1):
        """Pan by normalized percentage (-1.0…1.0)."""
        val = int(2048 + pct * 2047)
        spd = int(max(1, min(255, speed_pct * 255)))
        await self.send_cmd(0x02, val, spd)

    async def tilt_pct(self, pct: float, speed_pct: float = 0.1):
        """Tilt by normalized percentage (-1.0…1.0)."""
        val = int(2048 - pct * 2047)
        spd = int(max(1, min(255, speed_pct * 255)))
        await self.send_cmd(0x01, val, spd)

    async def pan_step(self, direction: str, step: int = 1, speed: int = 10):
        """Step pan by 'step' units: 'left' or 'right'."""
        match direction:
            case 'right':
                val = step
            case 'left':
                val = 4095 - step + 1
            case _:
                raise ValueError("Use 'left' or 'right'")
        await self.send_cmd(0x02, val, speed)

    async def tilt_step(self, direction: str, step: int = 1, speed: int = 10):
        """Step tilt by 'step' units: 'up' or 'down'."""
        match direction:
            case 'up':
                val = 2047 - step
            case 'down':
                val = 2048 + step
            case _:
                raise ValueError("Use 'up' or 'down'")

        await self.send_cmd(0x01, val, speed)

    async def stop(self):
        """Stop all motion immediately."""
        await asyncio.gather(
            self.send_cmd(0x02, 2048, 1),
            self.send_cmd(0x01, 2048, 1)
        )

    async def reset_position(self, speed_pct: float = 0.1):
        """Center gimbal: stop and hold central position."""
        await asyncio.gather(
            self.pan_pct(0.0, speed_pct),
            self.tilt_pct(0.0, speed_pct)
        )

    async def _motion_loop(self, cmd_id: int, value: int, speed: int, duration: float):
        if not self._client or not self._client.is_connected:
            raise ConnectionError("Not connected")

        count = max(1, int(duration / 0.2))

        for _ in range(count):
            await self.send_cmd(cmd_id, value, speed)
            await asyncio.sleep(0.2)

    def _on_notify(self, sender, data: bytearray):
        """Handles the notification packets received from the gimbal. Echo if enabled."""

        # `0x1815` -> Packet was sent from gimbal to app
        if len(data) >= 6 and data[4] == 0x18 and data[5] == 0x15:
            if self._heartbeat_enabled:
                asyncio.create_task(self._client.write_gatt_char(
                    self.write_uuid, data, response=False))
        else:
            cid = getattr(sender, 'uuid', str(sender))
            logger.info(f"[Notify] {cid}: {data.hex()}")

class Crane2SPresets:
    """
    High-level presets and tools for Crane2S.

    Examples:
      - pan_sweep
      - tilt_sweep
      - follow_path
      - track_2d
      - save and goto presets
    """
    def __init__(self, gimbal: Crane2S):
        self.gimbal = gimbal
        self.presets = {}

    async def pan_sweep(
        self,
        left_pct: float = -1.0,
        right_pct: float = 1.0,
        speed_pct: float = 0.1,
        dwell: float = 0.5,
        cycles: int = 1
    ):
        """Pan left→right repeatedly."""
        for _ in range(cycles):
            await self.gimbal.pan_pct(left_pct, speed_pct)
            await asyncio.sleep(dwell)
            await self.gimbal.pan_pct(right_pct, speed_pct)
            await asyncio.sleep(dwell)

    async def tilt_sweep(
        self,
        down_pct: float = -1.0,
        up_pct: float = 1.0,
        speed_pct: float = 0.1,
        dwell: float = 0.5,
        cycles: int = 1
    ):
        """Tilt down→up repeatedly."""
        for _ in range(cycles):
            await self.gimbal.tilt_pct(down_pct, speed_pct)
            await asyncio.sleep(dwell)
            await self.gimbal.tilt_pct(up_pct, speed_pct)
            await asyncio.sleep(dwell)

    async def follow_path(
        self,
        path: list[tuple[float, float]],
        speed_pct: float = 0.1,
        dwell: float = 0.1
    ):
        """Move through a list of (pan_pct, tilt_pct) points."""
        for pan, tilt in path:
            await self.gimbal.pan_pct(pan, speed_pct)
            await self.gimbal.tilt_pct(tilt, speed_pct)
            await asyncio.sleep(dwell)

    async def track_2d(
        self,
        target_x: float,
        target_y: float,
        frame_w: float,
        frame_h: float,
        speed_pct: float = 0.1
    ):
        """Center on (target_x, target_y) in frame dimensions."""
        pan_offset = (target_x / frame_w) * 2 - 1
        tilt_offset = 1 - (target_y / frame_h) * 2
        await self.gimbal.set_pan_pct(pan_offset, speed_pct)
        await self.gimbal.set_tilt_pct(tilt_offset, speed_pct)

    def save_preset(self, name: str, pan_pct: float, tilt_pct: float):
        """Store a named pan/tilt preset."""
        self.presets[name] = (pan_pct, tilt_pct)

    async def goto_preset(self, name: str, speed_pct: float = 0.1):
        """Move to a named preset."""
        pan, tilt = self.presets[name]
        await self.gimbal.pan_pct(pan, speed_pct)
        await self.gimbal.tilt_pct(tilt, speed_pct)

    async def reset_position(self, speed_pct: float = 0.1):
        """Alias for gimbal.reset_position."""
        await self.gimbal.reset_position(speed_pct)
