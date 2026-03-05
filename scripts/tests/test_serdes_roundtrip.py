import ctypes
import os
import pytest
import subprocess
import sys

# Build the shared library automatically
build_script_path = os.path.join(os.path.dirname(__file__), 'build_serdes_lib.sh')
repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    print("Building shared library...", file=sys.stderr)
    subprocess.run([build_script_path], check=True, cwd=repo_root, capture_output=True)
except subprocess.CalledProcessError as e:
    print(f"Failed to build shared library:\n{e.stderr.decode()}", file=sys.stderr)
    sys.exit(1)

# Load the shared libraries
lib_path = os.path.join(os.path.dirname(__file__), 'librbd_can_db.so')
lib_j1939_path = os.path.join(os.path.dirname(__file__), 'librbd_can_db_j1939.so')

try:
    ramn_can_db = ctypes.CDLL(lib_path)
except OSError:
    ramn_can_db = None

try:
    ramn_can_db_j1939 = ctypes.CDLL(lib_j1939_path)
except OSError:
    ramn_can_db_j1939 = None

if ramn_can_db:
    # Setup Default functions
    # ... (signatures are the same for both libraries, we can reuse setup logic)
    pass

def setup_lib_functions(lib):
    if not lib: return
    # void (uint16_t, uint8_t*)
    encode_funcs_16 = [
        "RAMN_Encode_Command_Brake", "RAMN_Encode_Control_Brake",
        "RAMN_Encode_Command_Accel", "RAMN_Encode_Control_Accel",
        "RAMN_Encode_Status_RPM", "RAMN_Encode_Command_Steering",
        "RAMN_Encode_Control_Steering", "RAMN_Encode_Command_TurnIndicator",
        "RAMN_Encode_Command_Sidebrake", "RAMN_Encode_Command_Lights",
    ]
    for func_name in encode_funcs_16:
        try:
            f = getattr(lib, func_name)
            f.argtypes = [ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8)]
            f.restype = None
        except AttributeError: pass

    # uint16_t (uint8_t*, uint32_t)
    decode_funcs_16 = [
        "RAMN_Decode_Command_Brake", "RAMN_Decode_Control_Brake",
        "RAMN_Decode_Command_Accel", "RAMN_Decode_Control_Accel",
        "RAMN_Decode_Status_RPM", "RAMN_Decode_Command_Steering",
        "RAMN_Decode_Control_Steering", "RAMN_Decode_Command_TurnIndicator",
        "RAMN_Decode_Command_Sidebrake", "RAMN_Decode_Command_Lights",
    ]
    for func_name in decode_funcs_16:
        try:
            f = getattr(lib, func_name)
            f.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
            f.restype = ctypes.c_uint16
        except AttributeError: pass

    # void (uint8_t, uint8_t*)
    encode_funcs_8 = [
        "RAMN_Encode_Command_Shift", "RAMN_Encode_Command_Horn",
        "RAMN_Encode_Control_Horn", "RAMN_Encode_Control_Sidebrake",
        "RAMN_Encode_Control_EngineKey", "RAMN_Encode_Control_Lights",
    ]
    for func_name in encode_funcs_8:
        try:
            f = getattr(lib, func_name)
            f.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)]
            f.restype = None
        except AttributeError: pass

    # void (uint8_t, uint8_t, uint8_t*)
    try:
        f = getattr(lib, "RAMN_Encode_Control_Shift_Joystick")
        f.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)]
        f.restype = None
    except AttributeError: pass

    # uint8_t (uint8_t*, uint32_t)
    decode_funcs_8 = [
        "RAMN_Decode_Command_Shift", "RAMN_Decode_Control_Shift",
        "RAMN_Decode_Joystick", "RAMN_Decode_Command_Horn",
        "RAMN_Decode_Control_Horn", "RAMN_Decode_Control_Sidebrake",
        "RAMN_Decode_Control_EngineKey", "RAMN_Decode_Control_Lights",
    ]
    for func_name in decode_funcs_8:
        try:
            f = getattr(lib, func_name)
            f.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
            f.restype = ctypes.c_uint8
        except AttributeError: pass

setup_lib_functions(ramn_can_db)
setup_lib_functions(ramn_can_db_j1939)

def test_ctypes_loaded():
    assert ramn_can_db is not None, "Failed to load librbd_can_db.so"
    assert ramn_can_db_j1939 is not None, "Failed to load librbd_can_db_j1939.so"

@pytest.mark.parametrize("signal_name, range_limit", [
    ("Command_Brake", 4096),
    ("Control_Brake", 4096),
    ("Command_Accel", 4096),
    ("Control_Accel", 4096),
    ("Status_RPM", 65536),
    ("Command_Steering", 65536),
    ("Control_Steering", 65536),
    ("Command_Shift", 256),
    ("Command_Horn", 256),
    ("Control_Horn", 256),
    ("Command_TurnIndicator", 65536),
    ("Command_Sidebrake", 65536),
    ("Control_Sidebrake", 256),
    ("Control_EngineKey", 256),
    ("Command_Lights", 65536),
    ("Control_Lights", 256),
])
def test_default_signal_roundtrip(signal_name, range_limit):
    encode_func = getattr(ramn_can_db, f"RAMN_Encode_{signal_name}")
    decode_func = getattr(ramn_can_db, f"RAMN_Decode_{signal_name}")
    step = 1 if range_limit <= 4096 else range_limit // 100
    for val in range(0, range_limit, step):
        payload = (ctypes.c_uint8 * 8)()
        encode_func(val, payload)
        decoded_val = decode_func(payload, 8)
        assert decoded_val == val, f"Roundtrip failed for {signal_name} at value {val}: got {decoded_val}"

@pytest.mark.parametrize("signal_name, range_limit, tolerance, used_bytes", [
    ("Command_Brake", 4096, 0, [0, 1]),
    ("Control_Brake", 4096, 20, [1]),
    ("Command_Accel", 4096, 0, [1, 2]),
    ("Control_Accel", 4096, 20, [1]),
    ("Status_RPM", 65536, 0, [3, 4]),
    ("Command_Steering", 65536, 0, [0, 1]),
    ("Control_Steering", 65536, 0, [0, 1]),
    ("Command_Shift", 130, 0, [2]),
    ("Command_Horn", 256, 0, [0]),
    ("Control_Horn", 256, 0, [0, 1]),
    # Command_TurnIndicator now uses specific RAMN mappings (0x0100, 0x0001) instead of 0-15, tested separately
    ("Command_Sidebrake", 4, 0, [0]),
    ("Control_Sidebrake", 4, 0, [0]),
    ("Control_EngineKey", 3, 0, [0]),
    ("Command_Lights", 256, 0, [0]),
])
def test_j1939_signal_roundtrip(signal_name, range_limit, tolerance, used_bytes):
    encode_func = getattr(ramn_can_db_j1939, f"RAMN_Encode_{signal_name}")
    decode_func = getattr(ramn_can_db_j1939, f"RAMN_Decode_{signal_name}")
    step = 1 if range_limit <= 4096 else range_limit // 100
    for val in range(0, range_limit, step):
        # Initialize payload with zeros to check if C code memsets to 0xFF
        payload = (ctypes.c_uint8 * 8)(0,0,0,0,0,0,0,0)
        encode_func(val, payload)
        
        # Isolation: Unused bytes should be 0xFF (set by C code)
        for i in range(8):
            if i not in used_bytes:
                assert payload[i] == 0xFF, f"Isolation failure for {signal_name} at byte {i}, val {val}: expected 0xFF, got 0x{payload[i]:02X}"
        
        decoded_val = decode_func(payload, 8)
        assert abs(int(decoded_val) - val) <= tolerance, f"J1939 Roundtrip failed for {signal_name} at value {val}: got {decoded_val}, expected {val} +/- {tolerance}"

def test_shift_joystick_roundtrip():
    encode_func = getattr(ramn_can_db, "RAMN_Encode_Control_Shift_Joystick")
    decode_shift = getattr(ramn_can_db, "RAMN_Decode_Control_Shift")
    decode_joystick = getattr(ramn_can_db, "RAMN_Decode_Joystick")

    for shift_val in [0, 127, 255]:
        for joy_val in [0, 127, 255]:
            payload = (ctypes.c_uint8 * 8)()
            encode_func(shift_val, joy_val, payload)
            assert decode_shift(payload, 8) == shift_val
            assert decode_joystick(payload, 8) == joy_val

def test_j1939_shift_joystick_roundtrip():
    encode_func = getattr(ramn_can_db_j1939, "RAMN_Encode_Control_Shift_Joystick")
    decode_shift = getattr(ramn_can_db_j1939, "RAMN_Decode_Control_Shift")
    decode_joystick = getattr(ramn_can_db_j1939, "RAMN_Decode_Joystick")

    for shift_val in [0, 64, 129]:
        for joy_val in [0, 64, 129]:
            payload = (ctypes.c_uint8 * 8)(0,0,0,0,0,0,0,0)
            encode_func(shift_val, joy_val, payload)

            # Isolation: bytes other than 3 and 4 should be 0xFF (set by merged memset)
            for i in range(8):
                if i not in [3, 4]:
                    assert payload[i] == 0xFF

            assert decode_shift(payload, 8) == shift_val
            assert decode_joystick(payload, 8) == joy_val

def test_j1939_turn_indicator_roundtrip():
    encode_func = getattr(ramn_can_db_j1939, "RAMN_Encode_Command_TurnIndicator")
    decode_func = getattr(ramn_can_db_j1939, "RAMN_Decode_Command_TurnIndicator")
    
    # 0: None, 0x0100: Left, 0x0001: Right, 0x0101: Hazard
    for val in [0, 0x0100, 0x0001, 0x0101]:
        payload = (ctypes.c_uint8 * 8)(0,0,0,0,0,0,0,0)
        encode_func(val, payload)
        
        for i in range(8):
            if i != 1:
                assert payload[i] == 0xFF
                
        assert decode_func(payload, 8) == val

def test_j1939_control_lights_turn_indicator():
    encode_func = getattr(ramn_can_db_j1939, "RAMN_Encode_Control_Lights")
    decode_func = getattr(ramn_can_db_j1939, "RAMN_Decode_Control_Lights")
    
    # Base lights (0x00, 0x08, 0x18, 0x38) combined with turn indicators (0x40 Left, 0x80 Right, 0xC0 Hazard)
    base_lights = [0x00, 0x08, 0x18, 0x38]
    turns = [0x00, 0x40, 0x80, 0xC0]
    
    test_cases = [l | t for l in base_lights for t in turns]
    
    for val in test_cases:
        payload = (ctypes.c_uint8 * 8)(0,0,0,0,0,0,0,0)
        encode_func(val, payload)
        
        for i in range(8):
            if i not in [0, 1]:
                assert payload[i] == 0xFF
                
        decoded_val = decode_func(payload, 8)
        assert decoded_val == val, f"Control_Lights roundtrip failed for {hex(val)}, got {hex(decoded_val)}"

