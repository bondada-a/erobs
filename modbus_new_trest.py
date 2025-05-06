from pymodbus.client.sync import ModbusTcpClient

ROBOT_IP = "192.168.1.10"  # Update this if needed
PORT = 502
KNOWN_IDS = [0, 1, 9, 16, 65, 100]

# Common OnRobot control/status register ranges
RANGES = [
    (0, 10),
    (1000, 10),
    (1020, 10),
    (1040, 10),
    (1050, 10),
]

client = ModbusTcpClient(ROBOT_IP, port=PORT)
if not client.connect():
    print("❌ Could not connect to Modbus TCP server")
    exit(1)

for unit_id in KNOWN_IDS:
    print(f"\n🔍 Checking unit ID {unit_id}")
    for base, count in RANGES:
        try:
            res = client.read_holding_registers(address=base, count=count, unit=unit_id)
            if not res.isError():
                print(f"  ✅ R{base}-{base+count-1}: {res.registers}")
            else:
                print(f"  ❌ R{base}-{base+count-1}: Modbus error")
        except Exception as e:
            print(f"  ⚠️ R{base}-{base+count-1}: {e}")

client.close()
