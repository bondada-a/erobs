from pymodbus.client import ModbusSerialClient
import time

client = ModbusSerialClient(
    port="/tmp/ttyUR",
    baudrate=115200,
    timeout=1,
    stopbits=1,
    bytesize=8,
    parity='N'
)

SLAVE_ID = 65
BASE_REG = 0x03E8  # 1000 decimal

def activate_gripper():
    print("➡️ Activating gripper...")
    control_word = 0b00000011  # ACT=1, GTO=1
    res = client.write_registers(address=BASE_REG + 0, values=[control_word, 0, 0], slave=SLAVE_ID)
    print("Activation:", res)

def move_gripper(pos, speed=255, force=50):
    print(f"➡️ Moving to {pos} (speed={speed}, force={force})...")
    control_word = 0b00000011  # ACT=1, GTO=1 (keep activated)
    res = client.write_registers(
        address=BASE_REG + 0,
        values=[control_word, pos, (speed << 8) | force],
        slave=SLAVE_ID
    )
    print("Move:", res)
def read_status():
    print("📡 Reading status...")
    res = client.read_holding_registers(address=BASE_REG + 3, count=3, slave=SLAVE_ID)
    if res.isError():
        print("❌ Read failed.")
    else:
        print(f"📍 Registers: {res.registers} (POS, STATUS, FAULT?)")

if client.connect():
    try:
        activate_gripper()
        time.sleep(1)

        move_gripper(0)   # open
        time.sleep(2)
        read_status()

        move_gripper(255)  # close
        time.sleep(2)
        read_status()

    finally:
        client.close()
        print("✅ Test complete.")
else:
    print("❌ Connection failed.")
