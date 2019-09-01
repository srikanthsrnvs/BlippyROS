from pymodbus.client.sync import ModbusSerialClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from constants import *


class Motor(object):

    def __init__(self, identifier, address, sync_client):
        self.identifier = identifier
        self.address = address
        self.sync_client = sync_client
        self.mode = "none"
        assert isinstance(identifier, str) and isinstance(address, int) and isinstance(sync_client, ModbusSerialClient)

    def get_position(self):
        pos_response = self.sync_client.read_holding_registers(MOTOR_POS, 2, unit=self.address)
        if pos_response.isError():
            return False
        decoder = BinaryPayloadDecoder.fromRegisters(pos_response.registers, byteorder=Endian.Big, wordorder=Endian.Big)
        return decoder.decode_32bit_int()

    def get_speed(self):
        speed_response = self.sync_client.read_holding_registers(MOTOR_SPEED, 1, unit=self.address)
        if speed_response.isError():
            return False
        return speed_response.registers[0]

    def get_torque(self):
        torque_response = self.sync_client.read_holding_registers(MOTOR_TORQUE, 1, unit=self.address)
        if torque_response.isError():
            return False
        return torque_response.registers[0]

    def get_acceleration(self):
        acc_response = self.sync_client.read_holding_registers(MOTOR_RAMP_ACC, 1, unit=self.address)
        if acc_response.isError():
            return False
        return acc_response.registers[0]

    def get_current_target(self):
        target_response = self.sync_client.read_holding_registers(SET_MOTOR_TARGET, 2, unit=self.address)
        if target_response.isError():
            return False
        decoder = BinaryPayloadDecoder.fromRegisters(target_response.registers, byteorder=Endian.Big, wordorder=Endian.Big)
        return decoder.decode_32bit_int()

    def get_max_speed(self):
        speed_response = self.sync_client.read_holding_registers(MOTOR_MAX_SPEED, 1, unit=self.address)
        if speed_response.isError():
            return False
        return speed_response.registers[0]

    def get_max_torque(self):
        torque_response = self.sync_client.read_holding_registers(MOTOR_MAX_TORQUE, 1, unit=self.address)
        if torque_response.isError():
            return False
        return torque_response.registers[0]

    def get_max_acceleration(self):
        acc_response = self.sync_client.read_holding_registers(MOTOR_MAX_ACCELERATION, 1, unit=self.address)
        if acc_response.isError():
            return False
        return acc_response.registers[0]

    def get_mode(self):
        mode = self.sync_client.read_holding_registers(MODE, 1, unit=self.address)
        if mode.isError():
            return False
        return mode.registers[0]

    def go_with_speed(self, speed_units, acc_units):
        # Check if speed mode, else set to speed and reset target
        read_mode = self.get_mode()
        if read_mode != 33:
            reset = self.reset_motor()
            if not reset:
                return False
            self.set_mode(33)
        self.set_max_acceleration(acc_units)
        return self.set_target(speed_units)

    def go_to_position(self, steps, speed_units, acc_units):
        # Check if position mode, else set to position and reset target
        read_mode = self.get_mode()
        if read_mode != 21:
            reset = self.reset_motor()
            if not reset:
                return False
            self.set_mode(21)
        self.set_max_speed(speed_units)
        self.set_max_acceleration(acc_units)
        return self.set_target(steps)

    def set_max_speed(self, steps):
        write_response = self.sync_client.write_register(MOTOR_MAX_SPEED, steps, unit=self.address)
        return not write_response.isError()

    def set_max_torque(self, torque):
        write_response = self.sync_client.write_register(MOTOR_MAX_TORQUE, torque, unit=self.address)
        return not write_response.isError()

    def set_max_acceleration(self, steps):
        write_response = self.sync_client.write_register(MOTOR_MAX_ACCELERATION, steps, unit=self.address)
        return not write_response.isError()

    def set_max_deceleration(self, steps):
        write_response = self.sync_client.write_register(MOTOR_MAX_DECELERATION, steps, unit=self.address)
        return not write_response.isError()

    def set_mode(self, mode):
        write_response = self.sync_client.write_register(MODE, mode, unit=self.address)
        return not write_response.isError()

    def set_target(self, target):
        builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        builder.add_32bit_int(target)
        request = self.sync_client.write_registers(SET_MOTOR_TARGET, builder.to_registers(), unit=self.address)
        return not request.isError()

    def reset_motor(self):
        write_response = self.sync_client.write_register(MODE, 1, unit=self.address)
        return not write_response.isError()

    def stop_motor(self):
        write_response = self.sync_client.write_register(MODE, 5, unit=self.address)
        return not write_response.isError()

    def get_current_status(self):
        motor_running_data = self.sync_client.read_holding_registers(MOTOR_POS, 4, unit=self.address)
        motor_voltage_data = self.sync_client.read_holding_registers(MOTOR_VOLTAGE, 1, unit=self.address)
        motor_current_data = self.sync_client.read_holding_registers(TORQUE_CURRENT, 1, unit=self.address)
        motor_operating_data = self.sync_client.read_holding_registers(TEMP_ELECTRONICS, 2, unit=self.address)
        motor_error_data = self.sync_client.read_holding_registers(ERROR, 1, unit=self.address)
        if motor_running_data.isError() or motor_error_data.isError() or motor_voltage_data.isError() or motor_current_data.isError() or motor_operating_data.isError():
            print("An error occured when attempting to read data. Please check USB address/connection, or check power to motors.")
            return False
        decoder = BinaryPayloadDecoder.fromRegisters(motor_running_data.registers[0:2], byteorder=Endian.Big,
                                                     wordorder=Endian.Big)
        decoder2 = BinaryPayloadDecoder.fromRegisters(motor_voltage_data.registers, byteorder=Endian.Big,
                                                     wordorder=Endian.Big)
        decoder3 = BinaryPayloadDecoder.fromRegisters(motor_current_data.registers, byteorder=Endian.Big,
                                                     wordorder=Endian.Big)
        decoder4 = BinaryPayloadDecoder.fromRegisters(motor_running_data.registers[3:4], byteorder=Endian.Big,
                                                      wordorder=Endian.Big)
        error = motor_error_data.registers[0]

        if error == 0:
            error = "No error"
        if error == 1:
            error = "General internal error"
        if error == 2:
            error = "Internal software timing error"
        if error == 3:
            error = "Error in application: code not terminating"
        if error == 4097:
            error = "General communication error"
        if error == 4098:
            error = "Invalid register error"
        if error == 4353:
            error = "Modbus parity error"
        if error == 4354:
            error = "Modbus framing error"
        if error == 4355:
            error = "Modbus overrun error"
        if error == 4356:
            error = "Modbus checksum error"
        if error == 4357:
            error = "Modbus illegal function code error"
        if error == 4358:
            error = "Modbus illegal diagnostics function code error"
        if error == 8193:
            error = "Hardware overcurrent protection triggered"
        if error == 12289:
            error = "Supply voltage too low"
        if error == 12290:
            error = "Supply voltage too high"
        if error == 16385:
            error = "Temperature of electronics is too high"
        if error == 16386:
            error = "Temperature of motor winding is too high"
        if error == 20481:
            error = "Torque limiting is active"
        if error == 24577:
            error = "Locked shaft condition detected"
        if error == 28673:
            error = "Regulator error is large"

        return {
            "position": decoder.decode_32bit_int(),
            "speed": motor_running_data.registers[2],
            "torque": decoder4.decode_16bit_int(),
            "voltage": decoder2.decode_16bit_int(),
            "current": decoder3.decode_16bit_int(),
            "electronics_temp": motor_operating_data.registers[0],
            "motor_temp": motor_operating_data.registers[1],
            "error": error
        }

