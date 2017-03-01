package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LEDSwitch extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Solenoid LEDSolenoid;
	
	private boolean isLEDOn = false;
	
	public LEDSwitch() {
		System.out.println("Instantiating LED");
		
		if (!RobotMap.isApollo) {
			LEDSolenoid = new Solenoid(RobotMap.LEDSWITCH_SOLENOID, RobotMap.LEDSWITCH_PCM);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setLEDOn() {
    	if (!RobotMap.isApollo) {
    		LEDSolenoid.set(true);
    	}
    }
    
    public void setLEDOff() {
    	if (!RobotMap.isApollo) {
    		LEDSolenoid.set(false);
    	}
    }
    
    public void toggleLED() {
    	if (isLEDOn) {
    		setLEDOff();
    	} else {
    		setLEDOn();
    	}
    	
    	isLEDOn = !isLEDOn;
    }
}

