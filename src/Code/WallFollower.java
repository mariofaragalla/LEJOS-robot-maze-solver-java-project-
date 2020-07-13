package Code;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class WallFollower {
	// Constants:
	// Distance constants:
	private final double MIN_DIST_TO_WALL = 0.25;// Minimum distance which indicates if a wall exists or no (25cm)
	private final double ONE_BLOCK_LENGTH = 0.385;// Length of one maze block (38.5cm)
	private final double COLOR_DETECT_DIST = 0.035;// Distance at which the robot detects colour (3.5cm)
	private final double DIST_TO_CENTER = 0.085;// Distance to keep the robot in centre from COLOR_DETECT_DIST(8.5cm)
	private final double DIST_BEFORE_WALL = 0.115;// Distance at which the robot should stand before the wall(11.5cm)
	// Speed constants:
	private final double SPEED = 250; // Constant robot speed throughout the maze
	private final int STOP_SPEED = 0;// Robot stopping speed
	// Sensors constants:
	private final long SENSOR_DELAY = 50; // Delay after which colour and ultrasonic sensors read data
	private final long GYRO_DELAY = 5; // Delay after which gyro sensor
	private final int OFFSET = 0; // Index at which any sensing array should store its data
	// Beep constants:
	private final int VOL_LEVEL = 1000; // Volume level
	private final int VOL_FREQUENCY = 1000;// beep frequency
	private final int VOL_DURATION = 1000;// beep duration
	// Gain constants:
	private final double KP_R = 8; // proportional gain for rotation
	private final double GEAR_RATIO = 3.0; // Gear transmission ratio between the robot's wheel and motor
	// Direction constants
	private final boolean CLOCKWISE = false; // clockwise direction
	private final boolean ANTICLOCKWISE = true;// counter clockwise direction
	private final boolean FORWARD = true; // Forward direction (ultrasonic sensor sensing direction)
	private final boolean BACKWARD = false; // Backward direction (wires direction)
	private final int RIGHT = 90; // indicates rotation to the right by 90 degrees
	private final int LEFT = 90;// indicates rotation to the left by 90 degrees
	private final int OPPOSITE_DIRECTION = 180;// indicates flipping of the whole robot 180 degrees
	// Robot parts constants
	private final double WHEEL_RADIUS = 0.0275;

	// Maze variables:
	private boolean deadend = true; // indicates if the robot reached a dead end (surrounded by 3 walls) 
	//assume that the robot will start in a dead end
	private boolean colorFound = false; // indicates if the robot found the colour selected from menu
	private int selectedColor = 0; // variable to store in the selected colour from menu

	//Sensors initialisation:
	//Ultrasonic sensor initialisation to port S4:
	private EV3UltrasonicSensor distanceSensor = new EV3UltrasonicSensor(SensorPort.S4); 
	private SampleProvider distance = distanceSensor.getDistanceMode();
	private float distArr[] = new float[1]; //array to fetch the ultrasonic sensor data in 
	//Colour sensor initialisation to port S1:
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private SensorMode color = colorSensor.getColorIDMode();
	private float colorReading[] = new float[1];//array to fetch the colour sensor data in 
	//Gyroscope sensor initialisation to port S3:
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	private SampleProvider gyroAngle = gyroSensor.getAngleMode();
	private float angleArr[] = new float[1];//array to fetch the Gyroscope sensor data in 
	//Motors initialisation (Right Motor to port C & Left Motor to port B):
	private EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	private EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	private EV3LargeRegulatedMotor syncList[] = { leftMotor }; // array for motor synchronisation
	
	private MazeMenu m =new MazeMenu(); // create a new menu for maze solving 
	
	public void solveMaze() { // method to solve the maze corresponding to a selected colour from the menu
		m.startMenu(); // start menu
		setEndColor(m.getMenuColor()); // get the colour that the robot should search for, from the menu.
		beep(); // beep to indicate start searching
		gyroSensor.reset(); // set the orientation of the robot at starting position to zero
		while (!colorFound) { // until the colour is found
			if (!deadend) { // if no dead end
				moveOneBlock(); // move one block
			} else { // if dead end found
				checkWallsColors(); // check the colours of the walls
				// if the robot didn't find the colour , then it should get out of the dead end
				deadend = false;
				if (colorFound) { // if the colour is found
					m.displayEndMessage(); // display the message of that should be displayed at the end
					Song s = new Song(); // create a new song
					s.playSuperMario(); // play super Mario song
					break; // stop searching for the colour
				}
			}
			setOrientation();
		}
	}

	public void setOrientation() { // the robot orientation before moving
		gyroSensor.reset(); // make current orientation zero
		double frontDistance = measureDistance(); // measure distance to the front wall
		rotate(LEFT, ANTICLOCKWISE); // rotate 90 degrees to the left anti-clockwise
		double leftDistance = measureDistance(); // measure distance to the left wall
		gyroSensor.reset(); // make current orientation zero
		if (leftDistance < MIN_DIST_TO_WALL) { // if left wall exists
			if (frontDistance > MIN_DIST_TO_WALL) { // if no front wall
				rotate(RIGHT, CLOCKWISE); // rotate 90 degrees to the right
			} else { // if front wall exists
				rotate(OPPOSITE_DIRECTION, CLOCKWISE); // rotate 180 degrees clockwise
				double rightDistance = measureDistance(); // measure distance to the right distance
				if (rightDistance < MIN_DIST_TO_WALL) { // if right wall exists
					rotate(RIGHT, CLOCKWISE); // rotate 90 degrees to the right
					// since left, front and right wall exists this means
					// that the robot reached a deadend
					deadend = true;
				}
			}
		}
	}

	public void moveOneBlock() { // move one block/tile in the maze
		// check if the distance in front of the robot is
		// more than the length of one block plus the distance before wall
		// i.e no wall will interrupt the robot to move the length of one block
		if (measureDistance() > ONE_BLOCK_LENGTH + DIST_BEFORE_WALL) {
			setMotorsSpeed(SPEED);
			moveDistance(ONE_BLOCK_LENGTH, FORWARD); // move one block distance
		} else { // if there is a wall that interrupts the robot from moving the total distance
			while (measureDistance() >= DIST_BEFORE_WALL) {
				move(SPEED); // keep moving until the robot reaches a certain distance from the wall
			}
			stop(); // stop after moving one block
		}
	}

	// move the robot a certain distance in a certain direction
	public void moveDistance(double distance, boolean direction) {
		if (direction == FORWARD) { // check if the direction is forward
			distance = -distance; // set distance to be negative
		}
		double wheelAngle = distance / WHEEL_RADIUS; // set the wheel angle to the given distance over the radius
		// multiply the wheel angle by transmission ratio then convert it to degrees
		// then assign it to the motor angle
		double motorAngle = Math.toDegrees(wheelAngle * GEAR_RATIO);
		// rotate the right and left motors with the motor angle corresponding to input
		// distance
		rightMotor.rotate((int) motorAngle, true);
		leftMotor.rotate((int) motorAngle, false);
	}

	public double measureDistance() { // measure the current distance of the ultrasonic sensor
		Delay.msDelay(SENSOR_DELAY);
		distance.fetchSample(distArr, OFFSET); // get current ultrasonic sensor reading
		Delay.msDelay(SENSOR_DELAY);
		return distArr[OFFSET]; // return the measured distance
	}

	// rotate the robot with a certain angle and direction
	public void rotate(int desiredAngle, boolean rotationDirection) {
		while (measureAngle() <= desiredAngle) { // until the robot reaches the desired angle
			double rotationError = getRotationError(desiredAngle); // get the rotation error
			if (rotationError <= 0) { // check if it is zero
				break; // end rotation
			}
			setRotationSpeed(rotationError); // set the robot speed based on the rotation error
			setRotDirection(rotationDirection); // set the direction of rotation of the robot and move w.r.t it
		}
		stop(); // stop the robot when it reaches the desired angle
	}

	public double measureAngle() { // get current robot orientation
		Delay.msDelay(GYRO_DELAY);
		gyroAngle.fetchSample(angleArr, OFFSET); // get current gyroscope reading
		Delay.msDelay(GYRO_DELAY);
		double measuredAngle = angleArr[OFFSET];
		return Math.abs(measuredAngle); // return the measured angle in positive
	}

	public double getRotationError(int desiredAngle) { // calculate the rotation error and return it
		// the rotation error is: the difference between the desired angle and
		// the angle which corresponds to the current orientation of the robot
		// Math.abs is used to get rid of the negative part
		double rotationError = Math.abs(desiredAngle - Math.abs(measureAngle()));
		return rotationError;
	}

	// set rotation speed corresponding to the rotation error and apply proportional
	// control
	public void setRotationSpeed(double rotationError) {
		rightMotor.setSpeed((int) (KP_R * rotationError));
		leftMotor.setSpeed((int) (KP_R * rotationError));
	}

	public void setRotDirection(boolean rotationDirection) { // set the rotation direction and move w.r.t this direction
		if (rotationDirection == ANTICLOCKWISE) { // if the rotation direction is anti-clockwise
			// move wheels with these directions
			rightMotor.backward();
			leftMotor.forward();
		} else { // if the rotation direction is clockwise
			// move wheels vice versa
			rightMotor.forward();
			leftMotor.backward();
		}
	}

	public void setMotorsSpeed(double speed) { // set the speed of the robot's motors
		rightMotor.setSpeed((int) speed);
		leftMotor.setSpeed((int) speed);
	}

	public void move(double speed) { // move the robot with given speed with motors synchronisation
		rightMotor.synchronizeWith(syncList);
		rightMotor.startSynchronization();
		setMotorsSpeed(speed);
		rightMotor.backward();
		leftMotor.backward();
		rightMotor.endSynchronization();
	}

	public void stop() { // Stop the motors with motors synchronisation
		rightMotor.synchronizeWith(syncList);
		rightMotor.startSynchronization();
		rightMotor.setSpeed(STOP_SPEED);
		leftMotor.setSpeed(STOP_SPEED);
		rightMotor.endSynchronization();
	}

	public void beep() { // make a beeeeep
		Sound.setVolume(VOL_LEVEL);
		Sound.playTone(VOL_FREQUENCY, VOL_DURATION);
	}

	public void checkWallsColors() { // Check the colours of all surrounding sides and return to original position
		for (int i = 0; i < 4; i++) { // repeat the following four times
			gyroSensor.reset();// reset the gyroscope
			if (measureDistance() < MIN_DIST_TO_WALL) { // If a wall exists
				checkWallColor(); // go and check the colour of this wall
			}
			if (!colorFound) { // if the selected colour is not found
				rotate(LEFT, ANTICLOCKWISE); // rotate to left direction anti-clockwise
			} else if (colorFound) { //if the selected colour is found
				stop(); // stop the robot and don't keep checking other walls' colours
				break;
			}
		}
	}

	public void checkWallColor() { // check the colour of one wall
		moveSmallDist(); // move forward until the colour detecting distance
		if (checkColor() == selectedColor) { // check if the colour is the same colour selected by the user
			colorFound = true; // set that the colour the robot is searching for, is found.
		}
		moveDistance(DIST_TO_CENTER, BACKWARD); // move backwards to the centre of the block
	}

	public void moveSmallDist() { // move a small distance (before the wall)
		while (measureDistance() >= COLOR_DETECT_DIST) { // keep measuring the distance until the robot
														//  reaches the colour detecting distance
			move(SPEED); // keep moving
			if (measureDistance() <= COLOR_DETECT_DIST) {  // check again to ensure that robot 
														   //reached its target before the loop ends
				break;
			}
		}
	}

	public int checkColor() { // check and return the number corresponding to the detected colour
		Delay.msDelay(SENSOR_DELAY);
		color.fetchSample(colorReading, OFFSET); // get colour reading
		Delay.msDelay(SENSOR_DELAY);
		return (int) colorReading[OFFSET]; // return colour reading
	}

	public void setEndColor(int selectedColor) { // set the colour at which the robot should end solving the maze
		this.selectedColor = selectedColor;
	}

}
