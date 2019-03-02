#Wheeled robot capable of steering, and using locks to protect data access
EV3 Mindstorm robot programmed with EV3RT

Description: We built a steerable robot according to our interpretation of the requirements given in HW4. Our robot incorporates the following sensors:
	- Ground Light Sensor: tracks the reflected value off the ground to identify the path boundary lines.
	- Ambient Light Sensor: tracks the value of the ambient light in the area to identify when the robot has gone under a table
	- Ultrasonic Sensor: tracks the distance to the nearest obstacle in front of the robot
	- Touch Sensor: tracks the number of collisions with obstacles
	
	The robot moves in a straight line until it encounters one of the blue lines indicating the boundaries of the path, at which point the robot will back up, turn in the direction of its previous turn, then move forward and straighten out after a short distance. If the robot encounters the blue line again while turning, it assumes that the direction of the turn was incorrect, and attempts to turn in the opposite direction, updating the assumed turn direction for the next turn.
	If the robot detects an obstacle at less than 8 cm, it will revers for a time, then attempt to dodge off the path to evade the obstacle, then rejoin the path. If the robot detects another obstacle or that the ambient light is too low, it will retreat back to its position when it started the evasive manuver, then dodge in the other direction.
	
	The system was composed of 4 tasks; one task for reading the ground light sensor, one for reading the ambient light sensor, one for reading the sonar sensor, and one for actuating the motors. Other functions, such as reading the touch sensor and communicating over bluetooth, are not done as part of the periodic schedule, and are done in main whenever there is idle time. 
	
	We calculated the worst case execution times for each task, as well as the critical sections for each task. For the 3 sensor tasks, their critical sections composed the entirety of their runtimes. The times are listed below, in microseconds:
	
	Sonar Task WECT 					= 99
	Light Ground Task WECT 				= 120
	Ambient Light Task WECT 			= 105
	Motor Task WECT						= 171
	Motor Task Critical Sections WECT 	= 75
	
	To make calculations easier, we approximated these times as:
	
	Sonar Task WECT 					= 100
	Light Ground Task WECT 				= 150
	Ambient Light Task WECT 			= 150
	Motor Task WECT						= 200
	Motor Task Critical Sections WECT 	= 75
	
	We calculated the blocking terms using the table method discussed in the class for the priority ceiling protocol, with this table:
	
	+----------------+-------------+--------------+------------+
	|                | Ground Lock | Ambient Lock | Sonar Lock |
	+----------------+-------------+--------------+------------+
	| light_ground() |     150     |       0      |      0     |
	+----------------+-------------+--------------+------------+
	|   light_amb()  |      0      |      150     |      0     |
	+----------------+-------------+--------------+------------+
	|     sonar()    |      0      |       0      |     100    |
	+----------------+-------------+--------------+------------+
	|    motors()    |      75     |      75      |     75     |
	+----------------+-------------+--------------+------------+
	
	So the blocking term for each sensor task is 75 microseconds, and for the motor task it is 0, since that task has the lowest priority.
	
	Computing the utilization, we did each separately, because of the blocking terms, using the rate-monotonic schedulability test:
	U1 = (150 + 75)/2000 = 0.1125 < 1
	U2 = 150/2000 + (150+75)/4000 = 0.13125 < 0.828
	U3 = 150/2000 + 150/4000 + (100 + 75)/8000 = 0.1345 < 0.779
	U4 = 150/2000 + 150/4000 + 100/8000 + (200 + 75)/16000 = 0.1422 < 0.756
	
	So this system is scheduleable, including the blocking created by the locks.
