import asyncio
import struct
from bleak import BleakClient
import logging

class Crane2S:
    """
    Crane 2S Gimbal controller over BLE using the reverse-engineered Zhiyun BTE protocol.

    Motion packets are 14 bytes:
    Byte Off.  Size  Field         Description
    --------- ----- ------------- ------------------------------
    0         1     0x24          Magic byte 1
    1         1     0x3C          Magic byte 2
    2–3       2     Length (LE)   Always 0x0008 for control packets
    4–5       2     Format ID     Always 0x1812 for app→gimbal
    6         1     Sequence ID   Increments each packet
    7         1     Direction     0x01 = app→gimbal
    8         1     Command ID    0x01=tilt, 0x02=pan, 0x03=roll
    9–10      2     Value (LE)    0–4095 (2048 = stop)
    11        1     Speed         0x01–0xFF
    12–13     2     CRC16/XMODEM  LE CRC over bytes 4–11
    """
    def __init__(self, address: str,
                 write_uuid: str = "d44bc439-abfd-45a2-b575-925416129600",
                 notify_uuid: str = "d44bc439-abfd-45a2-b575-925416129601"):
        self.address = address
        self.write_uuid = write_uuid
        self.notify_uuid = notify_uuid
        self._seq = 0
        self._client: BleakClient = None
        self._heartbeat_enabled = False

    async def connect(self, timeout: float = 10.0):
        """Connect and enable notifications + heartbeat echo."""
        self._client = BleakClient(self.address)
        await self._client.connect(timeout=timeout)
        if not self._client.is_connected:
            raise ConnectionError(f"Could not connect to {self.address}")
        if self.notify_uuid:
            await self._client.start_notify(self.notify_uuid, self._on_notify)
            self._heartbeat_enabled = True
        print(f"Connected to {self.address}")

    async def disconnect(self):
        """Stop notifications and disconnect."""
        if self._client and self._client.is_connected:
            if self.notify_uuid:
                self._heartbeat_enabled = False
                await self._client.stop_notify(self.notify_uuid)
            await self._client.disconnect()
            print(f"Disconnected from {self.address}")

    def _on_notify(self, sender, data: bytearray):
        """Handle incoming notifications; echo heartbeat, print others."""

        # As far as I know If the heartbeat is not echoed back, the gimbal will close the connection after sending
        # 4 heartbeats

        # Heartbeat format ID = 0x1815 at bytes 4–5
        if len(data) >= 6 and data[4] == 0x18 and data[5] == 0x15:
            if self._heartbeat_enabled:
                asyncio.create_task(self._echo_heartbeat(data))
        else:
            cid = getattr(sender, 'uuid', str(sender))
            print(f"[Notify] {cid}: {data.hex()}")

    async def _echo_heartbeat(self, raw: bytearray):
        """Echo raw heartbeat to keep connection alive."""
        try:
            await self._client.write_gatt_char(self.write_uuid, raw, response=False)
        except Exception as e:
            print(f"Heartbeat echo failed: {e}")

    def _xmodem_crc16(self, data: bytes) -> int:
        crc = 0x0000
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def _build_cmd(self, cmd_id: int, value: int, speed: int) -> bytearray:
        pkt = bytearray(b"\x24\x3C")
        pkt += (8).to_bytes(2, 'little')
        pkt += bytes.fromhex('1812')
        pkt += self._seq.to_bytes(1, 'little')
        self._seq = (self._seq + 1) & 0xFF
        pkt += b'\x01'
        pkt += cmd_id.to_bytes(1, 'little')
        pkt += (value & 0x0FFF).to_bytes(2, 'little')
        pkt += max(1, min(speed, 255)).to_bytes(1, 'little')
        crc = self._xmodem_crc16(pkt[4:])
        pkt += crc.to_bytes(2, 'little')
        return pkt

    async def _motion_loop(self, cmd_id: int, value: int, speed: int, duration: float):
        if not self._client or not self._client.is_connected:
            raise ConnectionError("Not connected to gimbal")
        interval = 0.2
        count = max(1, int(duration / interval))
        for _ in range(count):
            pkt = self._build_cmd(cmd_id, value, speed)
            print(f"→ {pkt.hex()}")
            await self._client.write_gatt_char(self.write_uuid, pkt, response=False)
            await asyncio.sleep(interval)

    async def pan(self, value: int, speed: int, duration: float = 1.0):
        """Pan: 0–2047 right, 2049–4095 left."""
        await self._motion_loop(0x02, value, speed, duration)

    async def tilt(self, value: int, speed: int, duration: float = 1.0):
        """Tilt: 0–2047 up, 2049–4095 down."""
        await self._motion_loop(0x01, value, speed, duration)

    async def pan_pct(self, percent: float, speed_pct: float, duration: float = 1.0):
        """Pan using percentage of full range."""
        val = int(percent * 4095)
        spd = int(speed_pct * 255)
        await self.pan(val, spd, duration)

    async def tilt_pct(self, percent: float, speed_pct: float, duration: float = 1.0):
        """Tilt using percentage of full range."""
        val = int(percent * 4095)
        spd = int(speed_pct * 255)
        await self.tilt(val, spd, duration)

