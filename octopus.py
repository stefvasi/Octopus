import serial
import struct
import threading
import time
import binascii
import atexit
from typing import Optional, Callable, Dict, List, Any, Tuple

# -------- SLIP Protocol Constants --------
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(payload: bytes) -> bytes:
    """Encode payload with SLIP protocol"""
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)


def slip_decode_stream(buf_obj: Dict, chunk: bytes) -> List[bytes]:
    """Decode SLIP stream, maintaining state in buf_obj"""
    frames = []
    for b in chunk:
        if b == SLIP_END:
            if buf_obj['data']:
                frames.append(bytes(buf_obj['data']))
                buf_obj['data'].clear()
            buf_obj['esc'] = False
            continue
        if buf_obj['esc']:
            if b == SLIP_ESC_END:
                b = SLIP_END
            elif b == SLIP_ESC_ESC:
                b = SLIP_ESC
            buf_obj['esc'] = False
        elif b == SLIP_ESC:
            buf_obj['esc'] = True
            continue
        buf_obj['data'].append(b)
    return frames




# -------- OSC Protocol Functions --------
def _pad4(b: bytes) -> bytes:
    """Pad bytes to 4-byte boundary"""
    return b + (b'\x00' * ((4 - (len(b) & 3)) & 3))

def osc_pack_message(addr: str, *args) -> bytes:
    """Pack an OSC message"""
    if not addr.startswith('/'):
        raise ValueError("OSC address must start with '/'")
    
    b_addr = _pad4(addr.encode('utf-8') + b'\x00')
    
    tags = [',']
    parts = [b_addr]
    
    for a in args:
        if isinstance(a, (bool, int)):
            tags.append('i')
        elif isinstance(a, float):
            tags.append('f')
        elif isinstance(a, str):
            tags.append('s')
        elif isinstance(a, (bytes, bytearray, memoryview)):
            tags.append('b')
        else:
            raise TypeError(f"Unsupported OSC arg type: {type(a)}")
    
    b_tags = _pad4(''.join(tags).encode('ascii') + b'\x00')
    parts.append(b_tags)
    
    for a, t in zip(args, tags[1:]):
        if t == 'i':
            parts.append(struct.pack('>i', int(a)))
        elif t == 'f':
            parts.append(struct.pack('>f', float(a)))
        elif t == 's':
            parts.append(_pad4(a.encode('utf-8') + b'\x00'))
        elif t == 'b':
            blob = bytes(a)
            parts.append(struct.pack('>i', len(blob)) + _pad4(blob))
    
    return b''.join(parts)

def _read_padded_str(buf: bytes, i: int) -> Tuple[str, int]:
    """Read null-terminated padded string from buffer"""
    j = buf.find(b'\x00', i)
    if j == -1:
        raise ValueError("OSC string without NUL")
    s = buf[i:j].decode('utf-8', errors='replace')
    i = i + ((j - i) + 4) & ~3
    return s, i

def _read_blob(buf: bytes, i: int) -> Tuple[bytes, int]:
    """Read blob from buffer"""
    if i + 4 > len(buf):
        raise ValueError("Blob size truncated")
    size = struct.unpack('>i', buf[i:i+4])[0]
    i += 4
    data = buf[i:i+size]
    i += (size + 3) & ~3
    return data, i

def parse_osc_message(buf: bytes, i: int = 0) -> Tuple[Tuple[str, List], int]:
    """Parse a single OSC message"""
    addr, i = _read_padded_str(buf, i)
    if not addr.startswith('/'):
        raise ValueError("Not an OSC address")
    
    tags, i = _read_padded_str(buf, i)
    if not tags or tags[0] != ',':
        raise ValueError("Missing typetags")
    
    args = []
    for t in tags[1:]:
        if t == 'i':
            args.append(struct.unpack('>i', buf[i:i+4])[0])
            i += 4
        elif t == 'f':
            args.append(struct.unpack('>f', buf[i:i+4])[0])
            i += 4
        elif t == 's':
            s, i = _read_padded_str(buf, i)
            args.append(s)
        elif t == 'b':
            b, i = _read_blob(buf, i)
            args.append(b)
        else:
            raise ValueError(f"Unsupported typetag: {t!r}")
    
    return (addr, args), i


def parse_osc_packet(pkt: bytes) -> List[Tuple[str, List]]:
    """
    Parse OSC packet (message or bundle) with validation and error recovery
    
    Returns list of (address, args) tuples
    """
    out = []
    
    # Empty packet
    if not pkt:
        return out
    
    # Validate and potentially recover from corruption
    if not (pkt.startswith(b'/') or pkt.startswith(b'#bundle\x00')):
        # Packet doesn't start correctly, try to find valid start
        slash_idx = pkt.find(b'/')
        bundle_idx = pkt.find(b'#bundle')
        
        if slash_idx >= 0 and (bundle_idx < 0 or slash_idx < bundle_idx):
            # Found a message start, skip corrupted bytes
            print(f"[ESP32] Recovered from corruption, skipping {slash_idx} bytes")
            pkt = pkt[slash_idx:]
        elif bundle_idx >= 0:
            # Found a bundle start
            print(f"[ESP32] Recovered from corruption, skipping {bundle_idx} bytes")
            pkt = pkt[bundle_idx:]
        else:
            # No valid OSC data found
            print(f"[ESP32] No valid OSC data in packet")
            return out
    
    # Parse based on packet type
    if pkt.startswith(b'#bundle\x00'):
        # Parse bundle
        if len(pkt) < 16:
            print(f"[ESP32] Bundle too short: {len(pkt)} bytes")
            return out
        
        # Skip '#bundle\0' (8 bytes) + timetag (8 bytes)
        i = 16
        element_count = 0
        
        while i < len(pkt):
            # Need at least 4 bytes for size
            if i + 4 > len(pkt):
                print(f"[ESP32] Bundle truncated at element {element_count}")
                break
            
            # Read element size (big-endian int32)
            try:
                size = struct.unpack('>i', pkt[i:i+4])[0]
            except struct.error:
                print(f"[ESP32] Failed to read size at position {i}")
                break
            
            i += 4
            
            # Validate size
            if size <= 0:
                print(f"[ESP32] Invalid element size: {size}")
                break
            
            if i + size > len(pkt):
                print(f"[ESP32] Element {element_count} extends beyond packet: need {size} bytes, have {len(pkt)-i}")
                break
            
            # Extract element
            elem = pkt[i:i+size]
            i += size
            element_count += 1
            
            # Validate element starts correctly
            if not elem:
                continue
                
            if not (elem.startswith(b'/') or elem.startswith(b'#bundle\x00')):
                print(f"[ESP32] Bundle element {element_count} invalid start: {elem[:8].hex()}")
                continue
            
            # Recursively parse element
            try:
                sub_msgs = parse_osc_packet(elem)
                out.extend(sub_msgs)
            except Exception as e:
                print(f"[ESP32] Failed parsing bundle element {element_count}: {e}")
                continue
        
        if element_count > 0 and len(out) == 0:
            print(f"[ESP32] Bundle had {element_count} elements but no valid messages")
            
    else:
        # Parse single message
        if not pkt.startswith(b'/'):
            print(f"[ESP32] Message doesn't start with '/': {pkt[:8].hex()}")
            return out
        
        try:
            # Find end of address string
            addr_end = pkt.find(b'\x00')
            if addr_end < 0:
                print(f"[ESP32] No null terminator in address")
                return out
            
            # Extract address
            addr = pkt[:addr_end].decode('utf-8', errors='replace')
            
            # Align to 4-byte boundary
            i = addr_end + 1
            while i % 4 != 0:
                i += 1
            
            # Check for typetag string
            if i >= len(pkt):
                print(f"[ESP32] Message truncated after address")
                return out
            
            if pkt[i:i+1] != b',':
                print(f"[ESP32] No typetag string at position {i}")
                return out
            
            # Find end of typetag string
            typetag_end = pkt.find(b'\x00', i)
            if typetag_end < 0:
                print(f"[ESP32] No null terminator in typetags")
                return out
            
            typetags = pkt[i:typetag_end].decode('ascii', errors='replace')
            
            # Align to 4-byte boundary
            i = typetag_end + 1
            while i % 4 != 0:
                i += 1
            
            # Parse arguments based on typetags
            args = []
            for tag in typetags[1:]:  # Skip the comma
                if i >= len(pkt):
                    print(f"[ESP32] Message truncated at arg {len(args)}")
                    break
                
                try:
                    if tag == 'i':
                        # Int32
                        if i + 4 > len(pkt):
                            print(f"[ESP32] Not enough bytes for int32")
                            break
                        args.append(struct.unpack('>i', pkt[i:i+4])[0])
                        i += 4
                        
                    elif tag == 'f':
                        # Float32
                        if i + 4 > len(pkt):
                            print(f"[ESP32] Not enough bytes for float32")
                            break
                        args.append(struct.unpack('>f', pkt[i:i+4])[0])
                        i += 4
                        
                    elif tag == 's':
                        # String
                        str_end = pkt.find(b'\x00', i)
                        if str_end < 0:
                            print(f"[ESP32] No null terminator for string arg")
                            break
                        s = pkt[i:str_end].decode('utf-8', errors='replace')
                        args.append(s)
                        # Align to 4-byte boundary
                        i = str_end + 1
                        while i % 4 != 0:
                            i += 1
                            
                    elif tag == 'b':
                        # Blob
                        if i + 4 > len(pkt):
                            print(f"[ESP32] Not enough bytes for blob size")
                            break
                        blob_size = struct.unpack('>i', pkt[i:i+4])[0]
                        i += 4
                        if i + blob_size > len(pkt):
                            print(f"[ESP32] Not enough bytes for blob data")
                            break
                        blob_data = pkt[i:i+blob_size]
                        args.append(blob_data)
                        # Align to 4-byte boundary
                        i += blob_size
                        while i % 4 != 0:
                            i += 1
                            
                    else:
                        print(f"[ESP32] Unsupported typetag: {tag}")
                        break
                        
                except Exception as e:
                    print(f"[ESP32] Error parsing arg {len(args)} (type {tag}): {e}")
                    break
            
            # Successfully parsed message
            out.append((addr, args))
            
        except Exception as e:
            print(f"[ESP32] Failed to parse message: {e}")
            print(f"  First 32 bytes: {pkt[:32].hex()}")
    
    return out


# -------- Main Serial SLIP-OSC Class --------
class SerialSlipOSC:
    """Handles SLIP-encoded OSC communication over serial"""
    
    def __init__(self, port: str, baud: int = 115200, 
                 on_msg: Optional[Callable[[str, List], None]] = None):
        self.port = port
        self.baud = baud
        self.on_msg = on_msg
        self.ser = None
        self._reader = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self.slip_buf = {'data': bytearray(), 'esc': False}
        self.state = {}  # Latest values by address
        self.is_running = False
    
    def open(self):
        """Open serial connection"""
        if self.ser:
            return
        
        try:
            # Try with exclusive flag (POSIX)
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0.05, exclusive=True
            )
        except TypeError:
            # Fallback for older pyserial
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        
        self.ser.reset_input_buffer()
        atexit.register(self.close)
    
    def start_reader(self):
        """Start the reader thread"""
        if not self.ser:
            self.open()
        if self._reader and self._reader.is_alive():
            return
        
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.is_running = True
    
    def close(self):
        """Close connection and stop reader"""
        self._stop.set()
        self.is_running = False
        
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None
    
    def _read_loop(self):
        """Main reader loop (runs in thread)"""

        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
                chunk = self.ser.read(n or 1)
                if not chunk:
                    continue
                
                frames = slip_decode_stream(self.slip_buf, chunk)
                for fr in frames:
                    try:
                        msgs = parse_osc_packet(fr)
                    except Exception as e:
                        # Debug output for bad packets
                        hx = binascii.hexlify(fr[:32]).decode()
                        print(f"[ESP32] Bad OSC: {e} len={len(fr)} hex={hx}...")
                        continue
                    
                    for addr, args in msgs:
                        self.state[addr] = args
                        if self.on_msg:
                            try:
                                self.on_msg(addr, args)
                            except Exception as cb_e:
                                print(f"[ESP32] Callback error: {cb_e}")
                                
            except serial.SerialException as se:
                print(f"[ESP32] Serial error: {se}")
                time.sleep(0.5)
            except Exception as e:
                print(f"[ESP32] Reader error: {e}")
                time.sleep(0.05)
    
    def send_osc(self, addr: str, *args):
        """Send OSC message"""
        payload = osc_pack_message(addr, *args)
        frame = slip_encode(payload)
        
        with self._write_lock:
            if self.ser:
                self.ser.write(frame)
                self.ser.flush()
    
    def send_haptic(self, idx: int, intensity_0to1: float, duration_ms: int):
        """Send haptic command matching ESP32 firmware format"""
        intensity_0to1 = max(0.0, min(1.0, float(intensity_0to1)))
        duration_ms = int(max(0, min(30000, duration_ms)))
        self.send_osc("/haptic", int(idx), float(intensity_0to1), int(duration_ms))
    
    def wait_for(self, addr: str, timeout: float = 2.0) -> Optional[List]:
        """Wait for specific OSC address to arrive"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if addr in self.state:
                return self.state[addr]
            time.sleep(0.01)
        return None