import serial
import serial.tools.list_ports
from atexit import register as register_exit
import matplotlib.pyplot as plotter
from matplotlib import colors as plotter_colors
from re import search

serial_handler = None

def at_exit():
	try:
		serial_handler.close()
	except AttributeError:
		pass

def on_close(event):
	exit()

register_exit(at_exit)

ports = serial.tools.list_ports.comports()

baudrate_list = (2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200)

# print available ports for selection
for pos, dev in enumerate(sorted(ports)):
		  print(f"{pos}. {dev[0]}: {dev[1]}")

serial_port = str(input("Select serial port to listen to>"))

try:
	# user can type port name ie. "COM1" or selection number
	if serial_port not in (sublist.device for sublist in ports):
		assert int(serial_port) > 0
		serial_port = ports[int(serial_port)]
except Exception:
	print(f"{serial_port} is not a valid selection.")
	input()
	exit(3)

# print preselected baudrates
print("Select baudrate:")
for option, value in enumerate(baudrate_list):
		print(f"{option}. {value}")
print(f"{len(baudrate_list)}. different")

baudrate = None

# check if baudrate is number
try:
	baudrate = int(input("Selection>"))
	assert baudrate > 0
	assert baudrate <= 10
except Exception:
	print(f"{baudrate} is not a valid selection.")
	input()
	exit(3)

# use preselected baudrate
if baudrate < len(baudrate_list):
	baudrate = baudrate_list[baudrate]
else:	# try user defined baudrate
	try:
		baudrate = int(input("Baudrate>"))
	except ValueError:
		print(f"{baudrate} is not a valid value.")
		input()
		exit(3)

# try to open serial port with selected user choice
try:
	serial_handler = serial.Serial(port=serial_port.device, baudrate=baudrate, timeout=0)
	print("OK")
except serial.SerialException as e:
	print(f"Interface error: {str(e).split(':')[0]}")
	input()
	exit(3)
except AttributeError:
	# try open user typed interface
	try:
		serial_handler = serial.Serial(port=serial_port, baudrate=baudrate, timeout=0)
		print("OK")
	except serial.SerialException as e:
		print(f"Interface error: {str(e).split(':')[0]}")
		input()
		exit(3)

graph = plotter.figure()
graph.canvas.mpl_connect('close_event', on_close)

plotter.xlabel(xlabel="sample")
plotter.ylabel(ylabel="value")
plotter.title(label="Serial data", weight="bold")
plotter.grid(visible=True, linestyle=':', linewidth=0.5)

data = []

print(f"Serial is open: {serial_handler.is_open}")

samples_amount = 0

try:
	samples_amount = int(input("Choose amount of samples to display to choose from\n>"))
	assert samples_amount > 0
	assert samples_amount <= 10
except ValueError:
	print(f"{samples_amount} is not a valid value.")
	input()
	exit(3)
except AssertionError:
	print(f"{samples_amount} should be in range (0, 10>.")
	input()
	exit(3)

samples = []

serial_handler.timeout = None

for i in range(samples_amount):
	line_raw = str(serial_handler.readline().decode('utf-8')).replace('\n', r'\n').replace('\t', '').replace('\r', '').replace("\x1b[2J", '').replace("\x1b[H", '')
	separator = search("[:\-=]", line_raw)
	line = None
	if separator:
		line = line_raw.split(line_raw[separator.start()])
		separator = line_raw[separator.start()]
	else:
		separator = None
		line = line_raw
	samples.append(
		{
			"sample_no": i,
			"separator": separator,
			"line_raw": line_raw,
			"parameter": line[0].strip() if len(line) > 1 else "Not found",
			"value": line[1].replace(r'\n', '').strip() if len(line) > 1 else "Not found"
		}
	)
	print(f"Sample: \t{i}\n"
	   		f"Separator: \t{samples[-1]['separator']}\n"
			f"Raw line: \t{samples[-1]['line_raw']}\n"
			f"Possible parameter: \t{samples[-1]['parameter']}\n"
			f"Possible value: \t{samples[-1]['value']}\n")

parameters = []

for sample in samples:
	parameter = sample['parameter']
	if parameter != "Not found":
		if parameter not in (sublist['parameter'] for sublist in parameters):
			parameters.append(
				{
					"parameter": sample['parameter'],
					"position": sample['sample_no'],
					"configured": False
				}
			)

print(f"Found following parameters: {', '.join(sublist['parameter'] for sublist in parameters)}\n")

configured = [sublist['configured'] for sublist in parameters]

while not all(configured):
	selection = str(input("Select parameter to configure (type \"\\exit\" to exit configure mode)\n>"))
	if selection == "\exit":
		print("Following parameters are not configured:")
		for no, is_configured in enumerate(configured):
			if not is_configured:
				print(f"{parameters[no]['parameter']}", end=', ' if no < len(configured) - 1 else '\n')
		choice = input("Are you sure want to exit? (N/y)>")
		if choice.lower() in ("yes", "y", "yep", "yeah"):
			break
	elif selection in tuple(sublist['parameter'] for sublist in parameters):
		label = input("Enter parameter label>")
		color = input("Enter color>")
		if color not in plotter_colors.cnames.keys() and not search("^#[0-9A-Fa-f]{6}$", color):
			print(f"{color} is not correct.")
			continue
		types = ("int", "uint", "hex", "bin")
		type = input(f"Parameter datatype ({', '.join(types)})>")
		if type not in types:
			print(f"{type} is not correct.")
			continue
		position = tuple(sublist['parameter'] for sublist in parameters).index(selection)
		configured[position] = True
		parameters[position]['configured'] = True
		
	else:
		print(f"{selection} is not a know parameter.\nTry again.")
		
input()

serial_handler.timeout = 0

while True:
	line = str(serial_handler.readline())
	line = line.split("=")
	if len(line) == 2:
		data.append(int(line[1].split("\\")[0].strip(), 0))
	if len(data) > 150:
		plotter.cla()
		data = []
	plotter.plot(data, color="red")
	plotter.draw()
	plotter.show(block=False)
	plotter.pause(0.001)
	
