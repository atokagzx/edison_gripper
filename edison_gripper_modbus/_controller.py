import time
from _modbus import ModbusRTUSlave, RegistersEnum
from _states import GripperStateRepresenter


class EdisonGripper:
    def __init__(self, port):
        self._port = port
        self._modbus = ModbusRTUSlave(port=self._port)
        self._gripper_state = GripperStateRepresenter()
        self._update_registers_from_state()
        self._modbus.start()

    
    def _update_registers_from_state(self):
        status_bytes = self._gripper_state.as_bytes()
        GRIPPER_STATUS_REGISTER = status_bytes[0:2]
        FAULT_AND_TARGET_POSITION_REGISTER = status_bytes[2:4]
        CURRENT_POSITION_AND_CURRENT_REGISTER = status_bytes[4:6]
        self._modbus[RegistersEnum.GRIPPER_STATUS_REGISTER] = GRIPPER_STATUS_REGISTER
        self._modbus[RegistersEnum.FAULT_AND_TARGET_POSITION_REGISTER] = FAULT_AND_TARGET_POSITION_REGISTER
        self._modbus[RegistersEnum.CURRENT_POSITION_AND_CURRENT_REGISTER] = CURRENT_POSITION_AND_CURRENT_REGISTER


if __name__ == "__main__":
    gripper = EdisonGripper(port='/dev/ttyUSB0')
    while True:
        time.sleep(0.1)
        gripper._update_registers_from_state()