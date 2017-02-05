package org.usfirst.frc.team3042.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static final int LEFT_JOYSTICK_USB_PORT_0 = 0;
	public static final int RIGHT_JOYSTICK_USB_PORT_1 = 1;
	public static final int GUNNER_JOYSTICK_USB_PORT_2 = 2;
	
	public static final int DRIVETRAIN_TALON_LEFT_FRONT = 19;
	public static final int DRIVETRAIN_TALON_LEFT_REAR = 11;
	public static final int DRIVETRAIN_TALON_RIGHT_FRONT = 14;
	public static final int DRIVETRAIN_TALON_RIGHT_REAR = 9;
	public static final int CLIMBER_TALON = 10;
	public static final int SHOOTER_TALON = 12;
	public static final int AGITATE_TALON = 3;
	public static final int INTAKE_TALON = 6;
	
	public static final int DRIVETRAIN_SOLENOID_SHIFT = 1;
	public static final int GDM_SOLENOID_ACTUATE = 3;
	
	public static final int COMPRESSOR_PORT = 0;
	
}
