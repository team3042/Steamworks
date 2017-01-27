package org.usfirst.frc.team3042.robot;
import org.usfirst.frc.team3042.robot.commands.Climber_Climb;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_ShiftGears;
import org.usfirst.frc.team3042.robot.commands.Shooter_Shoot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

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
	/*
	public Joystick joystickLeft = new Joystick(RobotMap.LEFT_JOYSTICK_USB_PORT_0);
	public Joystick joystickRight = new Joystick(RobotMap.RIGHT_JOYSTICK_USB_PORT_1);
	public Object left_1;
	public Object right_1;
	
	Button climberButton = new JoystickButton(joystickRight, 9);
	
	Button shooterButton = new JoystickButton(joystickRight, 8);
	
	Button gearShifter = new JoystickButton(joystickRight, 1);
	
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
	
	public OI(){
		climberButton.whileHeld(new Climber_Climb());
		
		shooterButton.whileHeld(new Shooter_Shoot());
		
		gearShifter.whenPressed(new DriveTrain_ShiftGears());
	}
	*/
}

