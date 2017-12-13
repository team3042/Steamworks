package org.usfirst.frc.team3042.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import edu.wpi.first.wpilibj.Servo;
/**
 *
 */
	
public class SoftwareBotServo extends Subsystem {
	Servo servo = new Servo(RobotMap.DRROBBY_SERVO);
	

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setPosition(double pos){// offical FRC servo libraries has a PWM range min .6 and max is 2.4, however the actual hardware specs are 0.553 and 2.450
		Robot.logger.log("setting position = "+ pos, 1);
		servo.set(convertServoPWM(pos));
    	//servo.set(pos);
    }
    
    private double convertServoPWM (double pos){//Conversion for HS-5685MH servo its hardware minPWM is .75 and a max of 2.25, Angle range is 0 to 116 degrees so don't be alarmed it wont reach exactly half way. 
    	double newMinPWM = .8;// both the min and max need some more guesswork so the servo will go to both sides equally else it was simply mounted wrong during the first test most likely the ladder.
    	double newMaxPWM = 2.2;
    	double convertedPos = (((newMaxPWM-newMinPWM)*pos)+(newMinPWM - 0.6))/1.8;
    	return convertedPos;
    }
}

