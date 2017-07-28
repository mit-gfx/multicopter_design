Author: Tao Du
Email: taodu@csail.mit.edu
Last updated: Jun 30, 2017

This folder contains the measurement data we collected in our experiments. You
don't have to use the exact model we list below to build your own copter, and
you should not seek to match your own measurement to ours as it is highly
dependent on the type of motors/propellers/batteries/speed controllers, etc. It
is STRONGLY recommended that you should do your own measurement BEFORE you
start to think about the controller and assemble a real copter.

List of components:
- Motor type: KDE2814XF-775 Brushless Motor.
- ESC type: Castle QuadPack 35 MultiRotor Controllers.
- Propeller size: 14x4.8/10x3.3 Carbon Fiber Propeller.
- Battery: Turnigy 2200mAh 3S 20C Lipo Pack.
- Device: RCbenchmark 1580.

Data files:
- motor_10inch_prop.txt/motor_14inch_prop.txt: The five columns are:
  - ESC signal: Range from 1000-2000.
  - Torque: In N times m.
  - Force: In N.
  - Motor speed: In RPM (Round per minute).
  - Current: In A.

Fitting measurement data:
Analytically, the torque is proportional to the force, and the force is
proportional to the square of the motor speed. Let T be the torque, F be the
force and w be the speed, we use linear least squares to find the best
coefficients c1 and c2 in T = c1 * F and F = c2 * w * w. We convert the unit of
w to radian/second before fitting them.