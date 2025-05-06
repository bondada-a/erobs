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
    print("❌ Failed to connect to /tmp/ttyUR")
    exit(1)

for slave_id in range(64, 66):
    print(f"🔍 Trying slave ID: {slave_id}")
    try:
        result = client.read_holding_registers(address=1000, count=1, slave=slave_id)
        if result.isError():
            continue
        else:
            print(f"✅ Found active Modbus device at ID {slave_id} → Value: {result.registers}")
    except Exception as e:
        print(f"Error on ID {slave_id}: {e}")

client.close()
