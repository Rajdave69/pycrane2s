"""
Crane2S Protocol Generator

This class provides a pure protocol implementation for constructing Crane 2S BLE control
packets without any Bluetooth dependencies. You can use this in any language or framework
to generate the exact byte payloads to send over your own BLE/UART/Socket layer.
"""

class Crane2SProtocol:
    """
    Stateless generator for Crane 2S motion command packets.

    Maintains an internal sequence ID that increments on each packet.

    Methods return a `bytes` object containing the 14-byte payload.
    """
    def __init__(self):
        # Sequence ID cycles 0x00–0xFF
        self._seq = 0

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    @staticmethod
    def _xmodem_crc16(data: bytes) -> int:
        """Compute CRC-16/XMODEM over the data."""
        crc = 0x0000
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def build_cmd(self, cmd_id: int, value: int, speed: int) -> bytes:
        """
        Build a raw Crane2S control packet.

        Args:
            cmd_id: 0x01=tilt, 0x02=pan, 0x03=roll
            value: 0–4095 (0x000–0xFFF), where 2048=center/stop
            speed: 1–255 (byte)

        Returns:
            14-byte command as bytes
        """
        seq = self._next_seq()
        # header
        pkt = bytearray([0x24, 0x3C])
        # length (little-endian 0x0008)
        pkt += (8).to_bytes(2, 'little')
        # format ID 0x1812
        pkt += bytes.fromhex('1812')
        # sequence
        pkt.append(seq)
        # direction: app→gimbal
        pkt.append(0x01)
        # command ID
        pkt.append(cmd_id & 0xFF)
        # value (12-bit, little-endian)
        pkt += (value & 0x0FFF).to_bytes(2, 'little')
        # speed byte
        pkt.append(max(1, min(255, speed)))
        # CRC over bytes 4–11
        crc = self._xmodem_crc16(bytes(pkt[4:]))
        pkt += crc.to_bytes(2, 'little')
        return bytes(pkt)

    # Convenience methods
    def pan(self, value: int, speed: int) -> bytes:
        """Pan: value<2048→right; >2048→left."""
        return self.build_cmd(0x02, value, speed)

    def tilt(self, value: int, speed: int) -> bytes:
        """Tilt: value<2048→up; >2048→down."""
        return self.build_cmd(0x01, value, speed)

    def pan_pct(self, pct: float, speed_pct: float) -> bytes:
        """Pan with pct in [-1.0,1.0]: -1 left, +1 right."""
        val = int(2048 + pct * 2047)
        spd = int(max(1, min(255, speed_pct * 255)))
        return self.pan(val, spd)

    def tilt_pct(self, pct: float, speed_pct: float) -> bytes:
        """Tilt with pct in [-1.0,1.0]: -1 down, +1 up."""
        val = int(2048 - pct * 2047)
        spd = int(max(1, min(255, speed_pct * 255)))
        return self.tilt(val, spd)

    def pan_step(self, direction: str, step: int = 1, speed: int = 10) -> bytes:
        """Step pan by `step` units in 'left' or 'right'."""
        if direction == 'right':
            val = step
        elif direction == 'left':
            val = 4095 - step + 1
        else:
            raise ValueError("direction must be 'left' or 'right'")
        return self.pan(val, speed)

    def tilt_step(self, direction: str, step: int = 1, speed: int = 10) -> bytes:
        """Step tilt by `step` units in 'up' or 'down'."""
        if direction == 'up':
            val = 2047 - step
        elif direction == 'down':
            val = 2048 + step
        else:
            raise ValueError("direction must be 'up' or 'down'")
        return self.tilt(val, speed)

    def stop(self) -> list[bytes]:
        """Generate stop commands for pan and tilt (centered)."""
        return [self.pan(2048, 1), self.tilt(2048, 1)]

    def reset_position(self, speed_pct: float = 0.1) -> list[bytes]:
        """Center gimbal to 0,0 with given speed percentage."""
        return [self.pan_pct(0.0, speed_pct), self.tilt_pct(0.0, speed_pct)]

# Example usage (no BLE):
# pb = Crane2SProtocol()
# pkt = pb.pan(16, 8)
# print(pkt.hex())
