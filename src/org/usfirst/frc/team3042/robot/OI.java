package org.usfirst.frc.team3042.robot;
import org.usfirst.frc.team3042.robot.commands.AutoMode_DriveForward;
import org.usfirst.frc.team3042.robot.commands.AutoMode_GearCenter;
import org.usfirst.frc.team3042.robot.commands.AutoMode_GearRight;
import org.usfirst.frc.team3042.robot.commands.AutoMode_HopperShootLeft;
import org.usfirst.frc.team3042.robot.commands.AutoTrajectory_BackFromLeft;
import org.usfirst.frc.team3042.robot.commands.AutoTrajectory_HopperToRight;
import org.usfirst.frc.team3042.robot.commands.Auto_Drive;
import org.usfirst.frc.team3042.robot.commands.Auto_LiftDrive;
import org.usfirst.frc.team3042.robot.commands.Climber_Climb;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_Calibrate;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_LiftAssist;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_ShiftGears;
import org.usfirst.frc.team3042.robot.commands.GDM_Actuate;
import org.usfirst.frc.team3042.robot.commands.GDM_OpenFront;
import org.usfirst.frc.team3042.robot.commands.Intake_Exhaust;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.LEDSwitch_SetOff;
import org.usfirst.frc.team3042.robot.commands.LEDSwitch_SetOn;
import org.usfirst.frc.team3042.robot.commands.Shooter_ReverseAgitator;
import org.usfirst.frc.team3042.robot.commands.Shooter_Shoot;
import org.usfirst.frc.team3042.robot.commands.Shooter_SpinUp;
import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;
import org.usfirst.frc.team3042.robot.commands.Auto_FollowTrajectory;
import org.usfirst.frc.team3042.robot.triggers.GamepadTrigger;
import org.usfirst.frc.team3042.robot.triggers.POVButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
	
	public Joystick joystickLeft = new Joystick(RobotMap.LEFT_JOYSTICK_USB_PORT_0);
	public Joystick joystickRight = new Joystick(RobotMap.RIGHT_JOYSTICK_USB_PORT_1);
	public Joystick gamepadGunner = new Joystick(RobotMap.GUNNER_JOYSTICK_USB_PORT_2);
	
	//Left Joystick Buttons
	public Button left_1 = new JoystickButton(joystickLeft, 1);
	public Button left_2 = new JoystickButton(joystickLeft, 2);
	public Button left_3 = new JoystickButton(joystickLeft, 3);
	public Button left_4 = new JoystickButton(joystickLeft, 4);
	public Button left_5 = new JoystickButton(joystickLeft, 5);
	public Button left_6 = new JoystickButton(joystickLeft, 6);
	public Button left_7 = new JoystickButton(joystickLeft, 7);
	public Button left_8 = new JoystickButton(joystickLeft, 8);
	public Button left_9 = new JoystickButton(joystickLeft, 9);

	//Right Joystick Buttons
	public Button right_1 = new JoystickButton(joystickRight, 1);
	public Button right_2 = new JoystickButton(joystickRight, 2);
	public Button right_3 = new JoystickButton(joystickRight, 3);
	public Button right_4 = new JoystickButton(joystickRight, 4);
	public Button right_5 = new JoystickButton(joystickRight, 5);
	public Button right_6 = new JoystickButton(joystickRight, 6);
	public Button right_7 = new JoystickButton(joystickRight, 7);
	public Button right_8 = new JoystickButton(joystickRight, 8);
	
	// Gamepad buttons
	Button gunner_A = new JoystickButton(gamepadGunner, 1);
	Button gunner_B = new JoystickButton(gamepadGunner, 2);
	Button gunner_X = new JoystickButton(gamepadGunner, 3);
	Button gunner_Y = new JoystickButton(gamepadGunner, 4);
	Button gunner_LB = new JoystickButton(gamepadGunner, 5);
	Button gunner_RB = new JoystickButton(gamepadGunner, 6);
	Button gunner_Back = new JoystickButton(gamepadGunner, 7);
	Button gunner_Start = new JoystickButton(gamepadGunner, 8);

	// Triggers
	Trigger gunner_LT = new GamepadTrigger(gamepadGunner,2);
	Trigger gunner_RT = new GamepadTrigger(gamepadGunner,3);
	Trigger gunner_LeftJoyUp = new GamepadTrigger(gamepadGunner, 1, GamepadTrigger.DIRECTION.UP);
	Trigger gunner_LeftJoyDown = new GamepadTrigger(gamepadGunner, 1, GamepadTrigger.DIRECTION.DOWN);
	Trigger gunner_POVUp = new POVButton(gamepadGunner, 0);
	Trigger gunner_POVDown = new POVButton(gamepadGunner, 180);
	Trigger gunner_POVLeft = new POVButton(gamepadGunner, 270);
	Trigger gunner_POVRight = new POVButton(gamepadGunner, 90);
	
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	
	public OI() {
		
		// Drivetrain
		right_1.whenPressed(new DriveTrain_ShiftGears());
		//right_7.whenPressed(new DriveTrain_Calibrate());
		left_1.whileHeld(new DriveTrain_LiftAssist());
		
		// GDM
		gunner_A.whenPressed(new GDM_Actuate());
		gunner_X.whenPressed(new GDM_OpenFront());
		
		// Climber
		gunner_POVUp.whileActive(new Climber_Climb());
		
		// Intake
		gunner_LT.whileActive(new Intake_Intake());
		gunner_LB.whileHeld(new Intake_Exhaust());
		
		// Shooter
		gunner_RT.whileActive(new Shooter_Shoot());
		gunner_RB.toggleWhenPressed(new Shooter_SpinUp(false));
		gunner_B.whileHeld(new Shooter_ReverseAgitator());
		
		// Phone Commands
		//gunner_POVLeft.whenActive(new Vision_TrackBoiler());
		//gunner_POVRight.whenActive(new Vision_TrackLift());
		
		// Software Testing
		left_7.whenPressed(new LEDSwitch_SetOn());
		left_8.whenPressed(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			true));
		left_9.whenPressed(new Auto_Drive(AutoType.TURN_RIGHT, -6, -5, 40));
	}
	
}

