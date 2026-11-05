"""Read one STM32 capture from a TCP stream, preserving recording boundaries."""
RECORDING_BYTES = 32 * 256 * 4

def receive_recording(connection):
    data = bytearray()
    while len(data) < RECORDING_BYTES:
        block = connection.recv(RECORDING_BYTES - len(data))
        if not block:
            if data:
                raise EOFError(f"Incomplete recording: {len(data)} of {RECORDING_BYTES} bytes")
            return None
        data.extend(block)
    return bytes(data)
