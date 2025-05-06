from pymodbus.client import ModbusSerialClient
import time

client = ModbusSerialClient(
    port="/tmp/ttyUR",
    baudrate=115200,
    timeout=0.5,
    stopbits=1,
    bytesize=8,
    parity='N'
)

control = 0x03       # ACT + GTO
position = 255         # Fully open
speed = 255
force = 50
speed_force = (speed << 8) | force

# Try all possible 3-register windows from 1000 to 1057
base_range = range(1000, 1058)

if not client.connect():
    print("❌ Could not connect")
    exit(1)

for base in base_range:
    try:
        print(f"📡 Trying write to base address {base} (regs {base}, {base+1}, {base+2})...")
        result = client.write_registers(
            address=base,
            values=[control, position, speed_force],
            slave=65
        )
        if result.isError():
            print(f"❌ Write to {base} failed")
        else:
            print(f"✅ Write to {base} accepted — watch for motion!")
        time.sleep(1.0)  # Wait to see if gripper moves
    except Exception as e:
        print(f"⚠️ Exception at address {base}: {e}")

client.close()
