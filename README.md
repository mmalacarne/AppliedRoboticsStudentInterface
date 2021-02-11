# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 


###Preliminary steps before real world arena application
In order to use the code in the real world arena it is necessary to perform the chessboard calibration.

1. Compile _camera\_calibration.cpp_
2. Run the executable passing _calib\_config.xml_ as argument. As a result it will be created _intrinsic\_calibration.xml_.
3. From _intrinsic\_calibration.xml_, copy the distortion_coefficients.
4. Go to line 139 in _src/extrinsicCalib.cpp_ and paste the values.
5. Go to line 11 and comment out the macro _DIST\_COEFFS\_DEFAULT_.

Last but not least, remember to comment out from all _.cpp_ the _DEBUG*_ macros.