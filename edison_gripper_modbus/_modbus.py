import serial
from enum import Enum
from threading import Thread
import time
import logging


class RegistersEnum(Enum):
    ACTION_AND_GRIPPER_OPTIONS_1_REGISTER = 0x03E8
    GRIPPER_OPTIONS_2_AND_POSITION_REQUEST_REGISTER = 0x03E9
    SPEED_AND_FORCE_REQUEST_REGISTER = 0x03EA
    # read only
    GRIPPER_STATUS_REGISTER = 0x07D0    
    FAULT_AND_TARGET_POSITION_REGISTER = 0x07D1
    CURRENT_POSITION_AND_CURRENT_REGISTER = 0x07D2


class ModbusCommandsEnum(Enum):
    READ_HOLDING_REGISTERS = 0x03
    READ_INPUT_REGISTERS = 0x04
    PRESET_MULTIPLE_REGISTERS = 0x10
    #TODO: Master read & write multiple registers FC23


class ValueCapacityError(ValueError):
    pass


class ModbusCommandsProcessor:
    def __init__(self, registers: dict, slave_id: int):
        '''
        supported_registers: dict of supported registers, key is the register address, value is the register content
        slave_id: slave id of the modbus device
        '''
        self._slave_id = slave_id
        self._registers = registers


    def process(self, data: bytes) -> bytes:
        '''
        data: data received from the modbus device
        '''
        if data[0] != self._slave_id:
            # raise ValueError(f'Received data from slave {data[0]}, expected slave {self._slave_id}')
            raise ValueError('Received data from slave {}, expected slave {}'.format(data[0], self._slave_id))
        try:
            command = ModbusCommandsEnum(data[1])
        except IndexError:
            raise ValueError('Received data is too short')
        except ValueError:
            # raise ValueError(f'Received unsupported command: {data[1]}')
            raise ValueError('Received unsupported command: {}'.format(data[1]))
        
        method_to_call = {
            ModbusCommandsEnum.READ_HOLDING_REGISTERS: self._read_holding_registers,
            ModbusCommandsEnum.READ_INPUT_REGISTERS: self._read_input_registers,
            ModbusCommandsEnum.PRESET_MULTIPLE_REGISTERS: self._preset_multiple_registers
        }[command]
        # print(f'Processing command {command}')
        response = method_to_call(data)
        # append CRC16
        response += self._compute_crc16(response)
        return response
    

    def _read_holding_registers(self, data: bytes) -> bytes:
        '''
        data: data received from the modbus device
        '''
        return self._read_registers_universal(data, self._registers)
        

    def _read_input_registers(self, data: bytes) -> bytes:
        '''
        data: data received from the modbus device
        '''
        return self._read_registers_universal(data, self._registers)
    

    def _preset_multiple_registers(self, data: bytes) -> bytes:
        '''
        data received from the modbus device
        '''
        if len(data) < 11:
            # raise ValueCapacityError(f'Received data has invalid length, contains {len(data)} bytes: {data}')
            raise ValueCapacityError('Received data has invalid length, contains {} bytes: {}'.format(len(data), data))
        start_address = int.from_bytes(data[2:4], 'big')
        number_of_registers = int.from_bytes(data[4:6], 'big')
        self._validate_register_address(start_address, number_of_registers)
        data_length = data[6]
        # check if data length is valid: 1 byte for number of registers, 2 bytes for each register
        if data_length != number_of_registers * 2:
            # raise ValueError(f'Received data has invalid length, contains {data_length} bytes: {data}')
            raise ValueError('Received data has invalid length, contains {} bytes: {}'.format(data_length, data))
        if len(data) != 9 + data_length:
            # raise ValueCapacityError(f'Received data has invalid length, contains {len(data)} bytes: {data}')
            raise ValueCapacityError('Received data has invalid length, contains {} bytes: {}'.format(len(data), data))
        self._check_crc16(data)
        # write data to registers
        for data_address, reg_address in enumerate(range(start_address, start_address + number_of_registers)):
            self._registers[reg_address] = int.from_bytes(data[7 + data_address * 2: 9 + data_address * 2], 'big')
        # return echo of the received data
        return bytes([self._slave_id, data[1], *data[2:8]])   


    def _read_registers_universal(self, data: bytes, registers: dict) -> bytes:
        '''
        data: data received from the modbus device
        registers: dict of registers to read, key is the register address, value is the register content
        '''
        if len(data) != 8:
            # raise ValueError(f'Received data has invalid length, contains {len(data)} bytes: {data}')
            raise ValueError('Received data has invalid length, contains {} bytes: {}'.format(len(data), data))
        self._check_crc16(data)
        # print(f'Received data: {data}')
        start_address = int.from_bytes(data[2:4], 'big')
        number_of_registers = int.from_bytes(data[4:6], 'big')
        self._validate_register_address(start_address, number_of_registers)
        response = bytes([self._slave_id, data[1], number_of_registers * 2])
        # print(bytes([self._slave_id]))
        # print(f'Response: {response}')
        for reg_address in range(start_address, start_address + number_of_registers):
            reg_value = registers[reg_address]
            if not isinstance(reg_value, bytes):
                reg_value = reg_value.to_bytes(2, 'big')
            response += reg_value
        return response


    def _validate_register_address(self, address: int, number_of_registers: int = 1):
        '''
        address: address of the first register to read
        number_of_registers: number of registers to read
        '''
        for reg_address in range(address, address + number_of_registers):
            if reg_address not in self._registers.keys():
                # raise ValueError(f'Register "{reg_address}" not supported')
                raise ValueError('Register "{}" not supported'.format(reg_address))
            
    
    def _compute_crc16(self, data: bytes) -> bytes:
        '''
        data: data to compute CRC16
        '''
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'big')
    

    def _check_crc16(self, data: bytes) -> bool:
        '''
        data: data to check CRC16
        '''
        if len(data) < 2:
            # raise ValueError(f'Received data is too short')
            raise ValueError('Received data is too short')
        crc = int.from_bytes(data[-2:], 'big')
        return crc == int.from_bytes(self._compute_crc16(data[:-2]), 'big')
    

class ModbusRTUSlave:
    def __init__(self, port, baudrate=115200, slave_id=9, timeout=1):
        self._port = port
        self._baudrate = baudrate
        self._slave_id = slave_id
        self._timeout = timeout
        self._last_time = 0
        self._registers = {}
        self._configure_registers()
        self._command_processor = ModbusCommandsProcessor(self._registers, self._slave_id)
        self._serial = None
        self._is_running = False


    def _configure_registers(self):
        self._registers[RegistersEnum.ACTION_AND_GRIPPER_OPTIONS_1_REGISTER.value] = 0
        self._registers[RegistersEnum.GRIPPER_OPTIONS_2_AND_POSITION_REQUEST_REGISTER.value] = 0
        self._registers[RegistersEnum.SPEED_AND_FORCE_REQUEST_REGISTER.value] = 0
        self._registers[RegistersEnum.GRIPPER_STATUS_REGISTER.value] = 0
        self._registers[RegistersEnum.FAULT_AND_TARGET_POSITION_REGISTER.value] = 0
        self._registers[RegistersEnum.CURRENT_POSITION_AND_CURRENT_REGISTER.value] = 0

    
    def start(self):
        self._serial = serial.Serial(self._port, self._baudrate, timeout=0)
        # self._serial.stopbits = serial.STOPBITS_ONE
        # self._serial.bytesize = serial.EIGHTBITS
        # self._serial.parity = serial.PARITY_NONE
        self._is_running = True
        self._cummulitive_data = b''
        self._thread = Thread(target=self._run, daemon=True, name='ModbusRTUSlaveReaderThread')
        # self._serial.reset_input_buffer()
        self._thread.start()


    def stop(self):
        self._is_running = False
        self._thread.join()
        self._serial.close()


    def _run(self):
        while self._is_running:
                # process received data
            data = self._serial.read(16)
            if len(data):
                if time.time() - self._last_time > self._timeout:
                    self._cummulitive_data = b''
                self._cummulitive_data += data
            else:
                # no data received, sleep for a while
                time.sleep(0.01)

            if len(self._cummulitive_data) >= 8:
                try:
                    response = self._command_processor.process(self._cummulitive_data)
                    # print(f'Response: {response} len: {len(response)}')
                    print('Request: {} len: {}'.format(self._cummulitive_data, len(self._cummulitive_data)))
                    print('Response: {} len: {}'.format(response, len(response)))
                    num_bytes = self._serial.write(bytes(response))
                    self._serial.flush()
                    print('Sent {} bytes'.format(num_bytes))
                except ValueCapacityError as e:
                    # print(f"value capacity error: {e}")
                    print('value capacity error: {}'.format(e))
                    continue
                except ValueError as e:
                    # print(f'Error processing received data: {e}')
                    print('Error processing received data: {}'.format(e))
                self._cummulitive_data = b''
            
    
    def __getitem__(self, key):
        if isinstance(key, Enum):
            key = key.value
        return self._registers[key]
    

    def __setitem__(self, key, value):
        if isinstance(key, Enum):
            key = key.value
        if key not in self._registers.keys():
            # raise ValueError(f'Register "{key}" not supported')
            raise ValueError('Register "{}" not supported'.format(key))
        self._registers[key] = value


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    slave = ModbusRTUSlave('/dev/ttyUSB0')
    slave.start()
    while True:
        # print(f'action: {slave[RegistersEnum.ACTION_AND_GRIPPER_OPTIONS_1_REGISTER]},\n'
        #       f'position: {slave[RegistersEnum.GRIPPER_OPTIONS_2_AND_POSITION_REQUEST_REGISTER]},\n'
        #       f'speed_force: {slave[RegistersEnum.SPEED_AND_FORCE_REQUEST_REGISTER]}')
        time.sleep(1)