import onRobot.gripper as gripper

rg_id = 0
ip = "192.168.1.10"
print("yayayaya")
rg_gripper = gripper.TwoFG7(ip,rg_id)

rg_width = rg_gripper.twofg_get_external_width()
# print("rg_width: ",rg_width)

target_force = 40

rg_gripper.twofg_grip_external(70.0, target_force)
