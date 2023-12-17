import struct
from typing import Any
import serial
from enum import Enum, IntEnum

class FixedEnum(IntEnum):
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_


class GripperActivationStatus(FixedEnum):
    RESET = 0
    ACTIVATION_IN_PROGRESS = 1


class GripperActionStatus(FixedEnum):
    STOPPED = 0
    GO_TO_REQUESTED_POSITION = 1


class GripperStatus(FixedEnum):
    RESET = 0
    ACTIVATION_IN_PROGRESS = 1
    ACTIVATION_COMPLETED = 3


class GripperObjectStatus(FixedEnum):
    FINGERS_IN_MOTION = 0
    CONTACT_WHILE_OPENING = 1
    CONTACT_WHILE_CLOSING = 2
    FINGERS_AT_REQUESTED_POSITION = 3


class GripperFaultStatus(FixedEnum):
    NO_FAULT = 0
    ACTION_DELAYED = 5
    ACTIVATION_BIT_MUST_BE_SET = 7
    MAXIMUM_OPERATING_TEMPERATURE_EXCEEDED = 8
    NO_COMMUNICATION_WITH_MASTER = 9
    UNDER_MINIMUM_OPERATING_VOLTAGE = 10
    AUTOMATIC_RELEASE_IN_PROGRESS = 11
    INTERNAL_FAULT = 12
    ACTIVATION_FAULT = 13
    OVERCURRENT_FAULT = 14
    AUTOMATIC_RELEASE_COMPLETED = 15


class GripperControllerFaultStatus(FixedEnum):
    DEFAULT = 0
   

class GripperStateRepresenter:
    def __init__(self):
        # robot input registers (read only)
        self._activation_status = GripperActivationStatus.RESET # gACT
        self._action_status = GripperActionStatus.STOPPED # gGTO
        self._gripper_status = GripperStatus.RESET # gSTA
        self._object_status = GripperObjectStatus.FINGERS_AT_REQUESTED_POSITION # gOBJ
        self._fault_status = GripperFaultStatus.ACTIVATION_BIT_MUST_BE_SET # gFLT
        self._controller_fault_status = GripperControllerFaultStatus.DEFAULT # kFLT
        self._requested_position = 0 # gPR
        self._actual_position = 0 # gPO
        self._actual_current = 0 # gCU


    @property
    def activation_status(self):
        '''
        (gACT) Activation status, echo of the rACT bit (activation bit).
        0: gripper reset
        1: activation in progress
        '''
        return self._activation_status
    

    @activation_status.setter
    def activation_status(self, value):
        if not GripperActivationStatus.has_value(value):
            # raise ValueError(f'activation_status must be in {GripperActivationStatus}')
            raise ValueError('activation_status must be in {}'.format(GripperActivationStatus))
        

    @property
    def action_status(self):
        '''
        (gGTO) Action status, echo of the rGTO bit (go to bit).
        0: stopped ( or performing activation / automatic release )
        1: go to requested position
        '''
        return self._action_status


    @action_status.setter
    def action_status(self, value):
        if not GripperActionStatus.has_value(value):
            # raise ValueError(f'action_status must be in {GripperActionStatus}')
            raise ValueError('action_status must be in {}'.format(GripperActionStatus))
        self._action_status = value


    @property
    def gripper_status(self):
        '''
        (gSTA) Gripper status, returns the current status & motion of the Gripper fingers
        0: gripper is in reset ( or automatic release ) state
        1: activation in progress
        2: not used
        3: activation is completed
        '''
        return self._gripper_status
    

    @gripper_status.setter
    def gripper_status(self, value):
        if not GripperStatus.has_value(value):
            # raise ValueError(f'gripper_status must be in {GripperStatus}')
            raise ValueError('gripper_status must be in {}'.format(GripperStatus))
        self._gripper_status = value


    @property
    def object_status(self):
        '''
        (gOBJ) Object detection status, is a built-in feature that provides information on possible object pick-up. Ignore if gGTO == 0.
        0: fingers are in motion towards requested position. No object detected.
        1: fingers stopped due to a contact while opening. Object detected.
        2: fingers stopped due to a contact while closing. Object detected.
        3: fingers are at requested position. No object detected or object lost.
        '''
        return self._object_status
    

    @object_status.setter
    def object_status(self, value):
        if not GripperObjectStatus.has_value(value):
            # raise ValueError(f'object_status must be in {GripperObjectStatus}')
            raise ValueError('object_status must be in {}'.format(GripperObjectStatus))
        self._object_status = value


    @property
    def fault_status(self):
        '''
        (gFLT) Fault status, returns the current fault status of the Gripper.
        0: no fault
        5: action delayed, initialization must be completed
        7: activation bit must be set prior to action
        8: maximum operating temperature exceeded
        9: no communication with the master (timeout 1 second)
        10: under minimum operating voltage
        11: automatic release in progress
        12: internal fault
        13: activation fault
        14: overcurrent fault
        15: automatic release completed
        '''
        return self._fault_status
    

    @fault_status.setter
    def fault_status(self, value):
        if not GripperFaultStatus.has_value(value):
            # raise ValueError(f'fault_status must be in {GripperFaultStatus}')
            raise ValueError('fault_status must be in {}'.format(GripperFaultStatus))
        self._fault_status = value


    @property
    def controller_fault_status(self):
        '''
        (kFLT) Controller fault status (not used)
        '''
        return self._controller_fault_status
    

    @controller_fault_status.setter
    def controller_fault_status(self, value):
        if not GripperControllerFaultStatus.has_value(value):
            # raise ValueError(f'controller_fault_status must be in {GripperControllerFaultStatus}')
            raise ValueError('controller_fault_status must be in {}'.format(GripperControllerFaultStatus))
        self._controller_fault_status = value


    @property
    def requested_position(self):
        '''
        (gPR) Echo of the requested position
        value: [0, 255]
        '''
        return self._requested_position
    

    @requested_position.setter
    def requested_position(self, value):
        if 0 <= value <= 255:
            raise ValueError('requested_position must be in [0, 255]')
        self._requested_position = value

    
    @property
    def actual_position(self):
        '''
        (gPO) Echo of the actual position
        value: [0, 255]
        '''
        return self._actual_position
    

    @actual_position.setter
    def actual_position(self, value):
        if 0 <= value <= 255:
            raise ValueError('actual_position must be in [0, 255]')
        self._actual_position = value


    @property
    def actual_current(self):
        '''
        (gCU) Echo of the actual current
        value: [0, 255]
        '''
        return self._actual_current
    

    @actual_current.setter
    def actual_current(self, value):
        if 0 <= value <= 255:
            raise ValueError('actual_current must be in [0, 255]')
        self._actual_current = value

    
    def as_bytes(self):
        gOBJ = (self._object_status << 6) & 0xC0
        gSTA = (self._gripper_status << 4) & 0x30
        gGTO = (self._action_status << 3) & 0x08
        gACT = (self._activation_status) & 0x01
        gripper_status_byte = gOBJ | gSTA | gGTO | gACT

        reserved_byte = 0

        kFLT = (self._controller_fault_status << 4) & 0xF0
        gFLT = (self._fault_status) & 0x0F
        fault_status_byte = kFLT | gFLT 
        
        gPR = (self._requested_position & 0xFF)
        position_echo_byte = gPR

        gPO = (self._actual_position) & 0xFF
        position_byte = gPO

        gCU = (self._actual_current & 0xFF)
        current_byte = gCU

        return struct.pack('BBBBBB', *[gripper_status_byte,
                                    reserved_byte,
                                    fault_status_byte,
                                    position_echo_byte,
                                    position_byte,
                                    current_byte])
    

if __name__ == '__main__':
    gripper_state = GripperStateRepresenter()
    print(gripper_state.as_bytes())
