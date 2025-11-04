# Wind-Tunnel

This project is a low-cost educational wind tunnel designed for aerodynamic testing.
It combines Arduino-based sensors with Python data visualization tools to provide real-time feedback on airflow, lift, drag, and environmental conditions.
The system is modular, so new sensors and calibration methods can be added as the project evolves.

Made by Lev and Ryan

**Use instructions:**

Plug the power supply into the outlet
Plug the USB into your laptop
Run "tunnel_gui.py"
Calibrate the tunnel if needed
Using the physical dial under the tunnel, set the fan power to match the desired speed
When the speed stabilizes, record the data into .csv
Calibration process:

**Calibration is done by applying 1 N of force to each load cell. The load cells are numbered in this way:**

Front vertical

Front horizontal

Back vertical

Back horizontal

However, if you mess up the order during the calibration, nothing will break, but you will have to stick to that new order.

**Structure of the project:**

tunnel_gui.py - GUI of the tunnel
serial_reader.py - reads the COM port and sends the data to GUI
calibration_matrix.txt - saves the calibration data between sessions
svgplot.py - produces the svg of the shape of the intake (needed for CAD design)
Arduino folder - code that is uploaded and executed on the arduino
Files to print folder - stl files of all parts that were 3d printed for the wind tunnel
