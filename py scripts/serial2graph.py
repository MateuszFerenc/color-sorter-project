import serial
import serial.tools.list_ports
from atexit import register as register_exit
import matplotlib.pyplot as plotter

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

for port, desc, hwid in sorted(ports):
		  print(f"{port}: {desc}")

serial_port = str(input("Select serial port to listen to>"))

print("Select baudrate:")
for option, value in enumerate(baudrate_list):
		print(f"{option}. {value}")
print(f"{len(baudrate_list)}. different")

baudrate = None

try:
	baudrate = int(input("Selection>"))
except ValueError:
	print(f"{baudrate} is not a valid selection.")
	input()
	exit(3)

if baudrate < len(baudrate_list):
	baudrate = baudrate_list[baudrate]
else:
	try:
		baudrate = int(baudrate)
	except ValueError:
		print(f"{baudrate} is not a valid value.")
		input()
		exit(3)

try:
	serial_handler = serial.Serial(port=serial_port, baudrate=baudrate)
	print("OK")
except serial.SerialException as e:
	print(f"Interface error: {str(e).split(":")[0]}")
	input()
	exit(3)

graph = plotter.figure()
graph.canvas.mpl_connect('close_event', on_close)

plotter.xlabel(xlabel="sample")
plotter.ylabel(ylabel="value")
plotter.title(label="Serial data", weight="bold")
plotter.grid(visible=True, linestyle=':', linewidth=0.5)
#plotter.ion()

data = []

print(f"Serial is open: {serial_handler.is_open}")

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
	
