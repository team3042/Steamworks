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
	
	public static final boolean isApollo = false;
	public static final boolean isPhone = true;
	
	public static final int LEFT_JOYSTICK_USB_PORT_0 = 0;
	public static final int RIGHT_JOYSTICK_USB_PORT_1 = 1;
	public static final int GUNNER_JOYSTICK_USB_PORT_2 = 2;
	
	public static final int DRIVETRAIN_TALON_LEFT_FRONT = (isApollo)? 19 : 17;
	public static final int DRIVETRAIN_TALON_LEFT_REAR = (isApollo)? 18 : 8;
	public static final int DRIVETRAIN_TALON_RIGHT_FRONT = (isApollo)? 14 : 7;
	public static final int DRIVETRAIN_TALON_RIGHT_REAR = (isApollo)? 9 : 15;
	public static final int CLIMBER_TALON = (isApollo)? 10 : 2;
	public static final int SHOOTER_TALON = (isApollo)? 3 : 5;
	public static final int AGITATE_TALON = (isApollo)? 6 : 4;
	public static final int INTAKE_TALON = (isApollo)? 12 : 1;
	
	public static final int DRIVETRAIN_SOLENOID_SHIFT = (isApollo)? 1 : 3;
	public static final int GDM_SOLENOID_ACTUATE = (isApollo)? 3 : 1;
	
	public static final int LEDSWITCH_PCM = 1;
	public static final int LEDSWITCH_SOLENOID = 1;
	
	public static final int COMPRESSOR_PORT = 0;
	
	
	
}
