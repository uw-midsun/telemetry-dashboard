"""
-------------------------------------------------------
Quick script to interpret and display telemetry information in console.
-------------------------------------------------------
Author:  Rebecca Cho
__updated__ = "2019-06-02"
-------------------------------------------------------
"""
import can
import cantools
import time

"""
-------------------------------------------------------
Constants
-------------------------------------------------------
"""
UNIDENTIFIED_CAN_MSGS = [25]

TEMP_THRESHOLD = 1
TOTAL_BATTERY_MODULES = 36
BATTERY_LAYOUT =   [35, 34, 33, 32, 31, 30, 
					24, 25, 26, 27, 28, 29,
					23, 22, 21, 20, 19, 18,
					 5,  4,  3,  2,  1,  0, 
					 6,  7,  8,  9, 10, 11,
					17, 16, 15, 14, 13, 12]

# ENUMERATIONS
DRIVE_OUTPUT_DIRECTION = {0:'NEUTRAL', 1:'FORWARD', 2:'REVERSE', 3:'Other'}

POWER_STATE = {0:'IDLE', 1:'CHARGE', 2:'DRIVE', 3:'Other'}


BPS_FAULT = {0:'KILLSWITCH', 1:'LTC AFE CELL', 2:'LTC AFE TEMP', 3:'LTC AFE FSM',\
			4:'LTC ADC', 5:'ACK TIMEOUT', 6:'Other'}
BPS_STATE = {0:'OK', 1:'FAULT-KILLSWITCH', 2:'FAULT-LTC AFE', 3:'FAULT-LTC AFE FSM',\
			4:'FAULT-LTC ADC', 5:'FAULT-ACK TIMEOUT'}
POWER_DISTRIBUTION_FAULT = {0:'BPS HB', 1:'BPS HB WATCHDOG', 2:'POWERTRAIN HB WATCHDOG',\
			3:'RELAY RETRY EXPIRY', 4:'SEQUENCE RETRY EXPIRY', 5:'Other'}


"""
-------------------------------------------------------
Global Variables
-------------------------------------------------------
"""
faulted = False

battery_time = None
battery_updated = False

# solar_front_time = None
# solar_front_updated = False
# solar_rear_time = None
# solar_rear_updated = False



# DRIVER CONTROL VARIABLES
throttle = direction = cruise_control_state = mechanical_brake_state = 0
cruise_control_target = 0
motor_velocity = motor_velocity_l = motor_velocity_r = 0

# BATTERY VARIABLES
min_v_module = min_t_module = max_v_module = max_t_module = 0

min_v = max_v = total_v = avg_v = voltage = 0
min_t = max_t = total_t = avg_t = temp = 0

module_v_count = module_t_count = 0

battery_list = [None]*TOTAL_BATTERY_MODULES

# BATTERY AGGREGATE VARIABLES
aggregate_voltage = aggregate_current = 0

# MOTOR CONTROLLER VARIABLES
mc_voltage_1 = mc_current_1 = 0
mc_voltage_2 = mc_current_2 = 0

# AUX DCDC VARIABLES
aux_voltage = aux_current = 0
dcdc_voltage = dcdc_current = 0

# SOLAR ARRAY VARIABLES
# ARRAY_LAYOUT = [[3, 2], [4, 1], [5, 0]]
TOTAL_ARRAY_MODULES_EACH = 6
array_front_modules = [None] * TOTAL_ARRAY_MODULES_EACH
array_rear_modules = [None] * TOTAL_ARRAY_MODULES_EACH
array_front_module_count = array_rear_module_count = 0

can_bus = can.interface.Bus('vcan0', bustype='socketcan')
db = cantools.database.load_file('system_can.dbc')


"""
-------------------------------------------------------
Functions
-------------------------------------------------------
"""
# 584
def read_drive_output(data): 
	global throttle, direction, cruise_control_state, mechanical_brake_state

	for key, value in data.items():
		if 'throttle' in key:
			throttle = value
		elif 'direction' in key:
			direction = value
		elif 'cruise_control' in key:
			cruise_control_state = value
		elif 'mechanical_brake_state' in key:
			mechanical_brake_state = value
	return

# 616
def read_cruise_control(data):
	global cruise_control_target

	for key, value in data.items():
		if 'target_speed' in key:
			cruise_control_target = value
	return

# 776
def read_lights_state(data):
	global light_state, light_id

	for key, value in data.items():
		if 'light_state' in key:
			light_state = value
		elif 'light_id' in key:
			light_id = value
	return

# 1025
def read_battery_vt(data):
	global min_v_module, min_t_module, max_v_module, max_t_module, module_v_count, module_t_count,\
			min_v, max_v, total_v, avg_v, voltage, min_t, max_t, total_t, avg_t, temp,\
			battery_list, battery_updated, battery_time

	for key, value in data.items():
		if 'BATTERY_VT_INDEX' in key:
			module = value + 1
		elif 'MODULE_VOLTAGE' in key:
			value /= 10000
			total_v += value
			voltage = value
			module_v_count += 1
		elif 'MODULE_TEMP' in key:
			value /= 1000
			temp = value
			if temp >= TEMP_THRESHOLD:
				total_t += value
				module_t_count += 1

	battery_list[module-1] = [voltage, temp]

	# note that battery messages may not always start at module 0
	if (module == 1 or min_v == 0):
		# new round of can messages
		# reset values
		total_v = total_t = avg_v = avg_t = 0
		# set min/max to first module
		min_v = max_v = voltage
		min_v_module = max_v_module = module
		min_t = max_t = temp
		min_t_module = max_t_module = module

		module_v_count = 1
		if temp > TEMP_THRESHOLD:
			module_t_count = 1
		else:
			module_t_count = 0

	else:
		if voltage < min_v:
			min_v = voltage
			min_v_module = module
		elif voltage > max_v:
			max_v = voltage
			max_v_module = module

		if temp > TEMP_THRESHOLD:
			if min_t < TEMP_THRESHOLD:
				min_t = temp
				min_t_module = module
			else:
				if temp < min_t:
					min_t = temp
					min_t_module = module
				elif temp > max_t:
					max_t = temp
					max_t_module = module


		# final module info received
		# output data if we have received info for each module
		if module == (TOTAL_BATTERY_MODULES):
			if module_v_count == 36:
				avg_v = total_v / module_v_count
				if module_t_count > 0:
					avg_t = total_t / module_t_count
				else:
					avg_t = 0
				battery_updated = True
				battery_time = time.time()

# 1057
def read_battery_aggregate_vc(data):
	global aggregate_voltage, aggregate_current

	for key, value in data.items():
		if 'voltage' in key:
			aggregate_voltage = value / 10000
		elif 'current' in key:
			aggregate_current = value / 1000000
	return

# 1127
def read_motor_controller_vc(data):
	global mc_voltage_1, mc_current_1, mc_voltage_2, mc_current_2

	for key, value in data.items():
		if 'mc_voltage_1' in key:
			mc_voltage_1 = value
		elif 'mc_current_1' in key:
			mc_current_1 = value
		elif 'mc_voltage_2' in key:
			mc_voltage_2 = value
		elif 'mc_current_2' in key:
			mc_current_2 = value
	return

# 1159
def read_motor_velocity(data):
	global motor_velocity

	for key, value in data.items():
		if 'vehicle_velocity_left' in key:
			motor_velocity_l = value
		elif 'vehicle_velocity_right' in key:
			motor_velocity_r = value
	motor_velocity = (motor_velocity_l + motor_velocity_r)/2 * 0.036
	return

# 1379
def read_aux_dcdc(data):
	global aux_voltage, aux_current, dcdc_voltage, dcdc_current

	for key, value in data.items():
		if 'aux_voltage' in key:
			aux_voltage = value / 1000
		elif 'aux_current' in key:
			aux_current = value / 1000
		elif 'dcdc_voltage' in key:
			dcdc_voltage = value / 1000
		elif 'dcdc_current' in key:
			dcdc_current = value / 1000
	return

# 1450
def read_solar_data_front(data):	# not used in this script
	global array_front_modules, array_front_module_count, solar_front_updated, solar_front_time

	for key, value in data.items():
		if 'SOLAR_SLAVE_INDEX' in key:
			array_module = value
		elif 'MODULE_VOLTAGE' in key:
			array_module_voltage = value / 10000
		elif 'MODULE_CURRENT' in key:
			array_module_current = value
		elif 'MODULE_TEMP' in key:
			array_module_temp = value / 1000

	array_front_modules[array_module-1] = [array_module_voltage, array_module_current, array_module_temp]

	if array_module == 0:
		array_front_module_count = 0
	array_front_module_count += 1

	if array_module == TOTAL_ARRAY_MODULES_EACH - 1:
		if array_front_module_count == 6:
			solar_front_updated = True
			solar_rear_time = time.time()

	return

# 1483
def read_solar_data_rear(data):		# not used in this script
	global array_rear_modules, array_rear_module_count, solar_rear_updated, solar_rear_time

	for key, value in data.items():
		if 'SOLAR_SLAVE_INDEX' in key:
			array_module = value
		elif 'MODULE_VOLTAGE' in key:
			array_module_voltage = value / 10000
		elif 'MODULE_CURRENT' in key:
			array_module_current = value / 1000
		elif 'MODULE_TEMP' in key:
			array_module_temp = value / 1000

	array_rear_modules[array_module-1] = [array_module_voltage, array_module_current, array_module_temp]

	if array_module == 0:
		array_rear_module_count = 0
	array_rear_module_count += 1

	if array_module == TOTAL_ARRAY_MODULES_EACH - 1:
		if array_rear_module_count == 6:
			solar_rear_updated = True
			solar_rear_time = time.time()

	return

def print_separator():
	print('===============================================================================================')
	return

def print_summary_data():
	print('Min voltage: {:6.3f}     Module: {}'.format(min_v, min_v_module), end='\t\t\t')
	print('Min temperature: {:6.3f}     Module: {}'.format(min_t, min_t_module))
	print('Max voltage: {:6.3f}     Module: {}'.format(max_v, max_v_module), end='\t\t\t')
	print('Max temperature: {:6.3f}     Module: {}'.format(max_t, max_t_module))
	print('Voltage discrepancy:\t{:6.3f}'.format(max_v - min_v), end='\t\t\t\t')
	print('Temperature discrepancy:\t{:6.3f}'.format(max_t - min_t))
	print('Average voltage:\t{:6.3f}\t\t\t\tAverage temperature:\t\t{:6.3f}\n'.format(avg_v, avg_t))
	print('Aggregate voltage:\t{:7.3f}\nAggregate current: \t{:7.3f}'.format(aggregate_voltage, aggregate_current))
	
	return

def print_battery_data():	#TODO: print time stamp
	module_counter = 0
	for i in BATTERY_LAYOUT:
		if (module_counter + 1) % 6 == 0:
			print(u'{:2d}: {:4.2f}V {:4.1f}C'.format(i+1, battery_list[i][0], battery_list[i][1]))
			if (module_counter+1 == 18):
				print('- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -')

		else:
			print(u'{:2d}: {:4.2f}V {:4.1f}C'.format(i+1, battery_list[i][0], battery_list[i][1]), end='|')
		module_counter+= 1
	return

def print_solar_array_data(): # TODO: test at next track day
	print("""
			 _______________   _____________	 ___________________________
			/ 	F3			| |	F2			|	| B2			B3			|
		   / 	V: {:6.3f}V | |	V: {:6.3f}V	|	| V: {:6.3f}V 	V: {:6.3f}V |
		  / 	C: {:6.3f}I | |	C: {:6.3f}I	|	| C: {:6.3f}I 	C: {:6.3f}I |
		 / 		T: {:6.3f}C | |	T: {:6.3f}C	|	| T: {:6.3f}C 	T: {:6.3f}C |
		|					| |				|	|							|
		|		F4			| |	F1			|	| B1			B4			|
		| 		V: {:6.3f}V | |	V: {:6.3f}V |	| V: {:6.3f}V 	V: {:6.3f}V |
		| 		C: {:6.3f}I | |	C: {:6.3f}I	|	| C: {:6.3f}I 	C: {:6.3f}I |
		| 		T: {:6.3f}C | |	T: {:6.3f}C	|	| T: {:6.3f}C 	T: {:6.3f}C |
		|					| |				|	|							|
		| 		F5			| |	F0			|	| B0			B5			|
		 \\ 	V: {:6.3f}V | |	V: {:6.3f}V	|	| V: {:6.3f}V 	V: {:6.3f}V	|
		  \\ 	C: {:6.3f}I | |	C: {:6.3f}I	|	| C: {:6.3f}I 	C: {:6.3f}I	|
		   \\ 	T: {:6.3f}C | |	T: {:6.3f}C	|	| T: {:6.3f}C 	T: {:6.3f}C	|
			\\______________| |_____________|	|___________________________|
		""".format(
			array_front_modules[3][0], array_front_modules[2][0], array_rear_modules[2][0], array_rear_modules[3][0],\
			array_front_modules[3][1], array_front_modules[2][1], array_rear_modules[2][1], array_rear_modules[3][1],\
			array_front_modules[3][2], array_front_modules[2][2], array_rear_modules[2][2], array_rear_modules[3][2],\
			array_front_modules[4][0], array_front_modules[1][0], array_rear_modules[1][0], array_rear_modules[4][0],\
			array_front_modules[4][1], array_front_modules[1][1], array_rear_modules[1][1], array_rear_modules[4][1],\
			array_front_modules[4][2], array_front_modules[1][2], array_rear_modules[1][2], array_rear_modules[4][2],\
			array_front_modules[5][0], array_front_modules[0][0], array_rear_modules[0][0], array_rear_modules[5][0],\
			array_front_modules[5][1], array_front_modules[0][1], array_rear_modules[0][1], array_rear_modules[5][1],\
			array_front_modules[5][2], array_front_modules[0][2], array_rear_modules[0][2], array_rear_modules[5][2],\
			))
	return	

def print_aux_dcdc_data():
	print('Aux Voltage: {:8.2f}\tAux Current: {:8}\tDCDC Voltage: {:8.2f}\tDCDC Current: {:8}'\
		.format(aux_voltage, aux_current, dcdc_voltage, dcdc_current))
	return

def print_driver_controls_data():
	print('Direction: {:7}  Throttle: {:7}  Brake State: {:7}  \nCruise Ctrl State: {:7}  Cruise Ctrl Target: {:7}'\
			.format(DRIVE_OUTPUT_DIRECTION.get(direction), throttle, mechanical_brake_state, cruise_control_state, cruise_control_target))
	print('Speed: {:9.2f}KMH'.format(motor_velocity))
	return

def print_motor_controller_data():
	print('MC Voltage 1: {}\tMC Current 1: {}'.format(mc_voltage_1, mc_current_1))
	print('MC Voltage 2: {}\tMC Current 2: {}'.format(mc_voltage_2, mc_current_2))
	return

def reset_values():
	# Reset variables corresponding with event-driven CAN messages. 
	global throttle, direction, cruise_control_state, mechanical_brake_state,\
			cruise_control_target, motor_velocity

	throttle = 0
	direction = 0
	cruise_control_state = 0
	mechanical_brake_state = 0
	cruise_control_target = 0
	motor_velocity = 0


def main():
	global battery_updated #, solar_rear_updated, solar_front_updated

	while True:
		can_message = can_bus.recv()
		msg_id = can_message.arbitration_id
		if msg_id not in UNIDENTIFIED_CAN_MSGS:
			data = db.decode_message(can_message.arbitration_id, can_message.data)
			if (msg_id == 35):	# FAULT
				# TODO: change this to a try/catch statement
				print(data)

			else:
				if (msg_id == 584):		# DRIVE OUTPUT
					read_drive_output(data)

				elif (msg_id == 616):	# CRUISE CONTROL TARGET
					read_cruise_control(data)

				elif (msg_id == 1025 and not battery_updated):	# BATTERY VT
					read_battery_vt(data)

				elif (msg_id == 1057):	# BATTERY AGGREGATE VC
					read_battery_aggregate_vc(data)

				elif (msg_id == 1127):	# MOTOR CONTROLLER VC
					read_motor_controller_vc(data)

				elif (msg_id == 1159):	# MOTOR VELOCITY
					read_motor_velocity(data)

				elif (msg_id == 1379):	# AUX DCDC
					read_aux_dcdc(data)

				elif (msg_id == 1450): 	# SOLAR DATA FRONT
					read_solar_data_front(data)

				elif (msg_id == 1483):	# SOLAR DATA REAR
					read_solar_data_rear(data)


			if battery_updated:	# and solar_rear_updated and solar_front_updated:
				print('\n--------------------------------------------SUMMARY--------------------------------------------')
				print_summary_data()
				print_separator()
				print_aux_dcdc_data()
				print_separator()
				print('--------------------------------------------BATTERY--------------------------------------------')
				print_battery_data()
				print_separator()
				print_driver_controls_data()
				print_separator()
				print_motor_controller_data()
				print_separator()
				#print('-----------------------------------------ARRAY-----------------------------------------')
				#print_solar_array_data()
				#print_separator()
				battery_updated = False # solar_front_updated = solar_rear_updated = False
				reset_values()

if __name__ == '__main__':
	main()

					
