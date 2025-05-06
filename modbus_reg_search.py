from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(
    port="/tmp/ttyUR",
    baudrate=115200,
    timeout=0.5,
    stopbits=1,
    bytesize=8,
    parity='N'
)

if not client.connect():
    print("❌ Could not connect to gripper")
    exit(1)

print("🔍 Scanning registers 1020–1060 (likely control/status block):")

try:
    res = client.read_holding_registers(address=1020, count=40, slave=65)
    if not res.isError():
        for i, val in enumerate(res.registers):
            print(f"R{1020 + i}: {val}")
    else:
        print("❌ Error reading registers")
except Exception as e:
    print(f"⚠️ Exception during read: {e}")

client.close()
