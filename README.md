# titanCode
<<<<<<< HEAD
Code on titan 6

Included Packages:
	py_con - python controls with IK, joystick, stand/sit
	eigenbot - contains eigenbot_drivers
	titan_base - simple joint space commands for small demos
=======

	OPERATING TITAN6

	Batteries:
		Main batteries - Nominal voltage 50 v, can charge up to 5 amps
		Computer battery (LiGo) - Nominal voltage 36 v, can charge up to 3 amps

	E stop (on the leg) - controls power to modules
	M stop (wireless) - prevents commanded movements 

	Module Lights:
		M stop engaged : yellow and red flashing
		M stop disengaged : blue and green flashing
		Encoder problem : red flashing

	Running Smart Startup Script:
	
		Plug in LiGo, and turn it on. Also disengage E stop to turn on robot. 

		Make sure Joints are zeroed (keep M stop Engaged)

		From a terminal
                                ssh biorobotics@titanuc
                                (password is "biorobotics")
		
		Navigate to titanCode folder

		run this command
			./runPyCon.sh
			(Should take 8 seconds to startup)
		
		Disengage M stop and control normally

	Running Joystick script Manually:

		Power On: 

			Plug LiGo battery and switch it on (There are two anderson connectors that need to be connected)

			With  M stop engaged, disengage E stop

		Connecting to onboard computer:

			TITAN wifi network will show up once the LiGo has been plugged in and powered on. Connect to TITAN from your computer
				Password: biorobotics
			(TERMINAL 1) From a terminal
				ssh biorobotics@titanuc
				(password is "biorobotics")
				(Proper directories are sourced on startup for running joystick control)

			(TERMINAL 1) Next run
				roslaunch eigenbot_driver titan_driver.launch
				(this launch the controller that allows the modules to be commanded)


		Zeroing Joints:

			With robot powered off or M stopped (it is easier to move the joints if there are no power to the modules), move all joints so that shins of robot are vertical and all waist (horizontal) joints are centered. (Note that you can turn off the modules by pressing the e stop without turning off the computer)

			Make sure the modules are on and M stopped if you turned them off in the previous step

			(TERMINAL 2) Open a new terminal and ssh into the robot computer (see "connecting to onboard computer")

			(TERMINAL 2) from this terminal run
				rostopic echo /titan/joint/fb
				(this will display all module feedback)
				(titan_driver must be launched for this comand to display module feedback)

			(TERMINAL 3) Open a new terminal and ssh into the robot computer (see "connecting to onboard computer")

			(TERMINAL 3) from this terminal navigate to titanCode
				cd titanCode

			(TERMINAL 3) from titanCode run
				./zeroJoints
				(this should set all of the module positions to zero)

			(TERMINAL 2) Look at the previous terminal and confirm that all position values are reading zero
				(some of the values will not be zero as they are the force values from the foot sensors. You can distinguish these as the corresponding index of the "name" array will be a number in the 60s)
				(Mostly we are checking to make sure none of the joints are randomly reading large numbers)

			If joint values look good after zeroing, you can disengage the M stop, otherwise, run ./zeroJoints again and check the values
				(toggling the e stop may also help if joints are still acting up)

		Running Joystick Control:

			Once titan_drivers are launched and joints are zeroed, you are ready to run Joystick Control!

			Turn on the joystick by pressing the big button in the middle 
				(the top left and top right lights should be on)
				(pressing the center button will toggle control modes)

			(TERMINAL 4) Open a new terminal and ssh into the robot computer (see "connecting to onboard computer")

			(TERMINAL 4) Now launch the ros joystick node to allow for the joystick to communicate with the robot
				rosrun joy joy_node
				(you can check to see if the joystick is sending signal by running rostopic echo joy (use another terminal for this))

			(TERMINAL 5) Open a new terminal and ssh into the robot computer (see "connecting to onboard computer")

			(TERMINAL 5) from this terminal navigate to py_con
				cd titanCode/src/py_con/

			(TERMINAL 5) from py_con run
				python3 titan.py ROS
				(this will run the controller)

		Joystick Controls:

			X - stand up/sit down
			L stick - strafe
			R stick - turn
			(EigenBus is facing forward wrt the robot)
			(Small commands go a long way!)
			(O6 is very large and will take a while to complete a step once you tell it to)

		Powering Off:

			(TERMINAL 5) Use control C while selecting the terminal running titan.py to stop the controller

			From any terminal that is SSHed and has nothing running, run
				sudo halt
				(password "biorobotics")

			Engage E stop to power off Modules

			Turn off LiGo battery

		Diagnosing Problems:

			If modules turns red, power cycling the robot should fix this 
			Leg bending inward - If you see a leg bending inward under the center body, try to support the robot best you can and M stop. This position can easily cause the leg to snap





>>>>>>> 46da52e7eb883c0918509ee8ec81c5d57fe29754
