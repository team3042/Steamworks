package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GDM extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Relay GDMRelay = new Relay(RobotMap.GDM_RELAY);
	boolean isActive = false;
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void actuate(){
    	
    	if(isActive){
    		GDMRelay.set(Relay.Value.kOff);
    		isActive = false;
    	}
    	else{
    		GDMRelay.set(Relay.Value.kOn);
    		isActive = true;
    	}
    }
}

