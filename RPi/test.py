import os
import can
import time

msg_engine_cu = can.Message(arbitration_id=0x1, is_remote_frame=True, is_extended_id=False)
msg_transmission_cu = can.Message(arbitration_id=0x45, is_remote_frame=True, is_extended_id=False)
msg_speed_cu = can.Message(arbitration_id=0x10, is_remote_frame=True, is_extended_id=False)
msg_brake_cu = can.Message(arbitration_id=0x5, is_remote_frame=True, is_extended_id=False)
msg_door_cu = can.Message(arbitration_id=0x134, is_remote_frame=True, is_extended_id=False)
msg_seat_cu = can.Message(arbitration_id=0x176, is_remote_frame=True, is_extended_id=False)
msg_hmi = can.Message(arbitration_id=0x200, is_remote_frame=True, is_extended_id=False)

try:
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    can0 = can.interface.Bus(channel = 'can0', bustype='socketcan', receive_own_messages=False)

    while True:
        for message in [msg_engine_cu, msg_transmission_cu, msg_speed_cu, msg_brake_cu, msg_door_cu, msg_seat_cu, msg_hmi]:
            can0.set_filters([{"can_id": message.arbitration_id, "can_mask": 0xFFFF}])
            can0.send(message)
            msg = can0.recv(1000.0)
            print msg
            time.sleep(0.1)
            
except KeyboardInterrupt:
    os.system('sudo ip link set can0 down')