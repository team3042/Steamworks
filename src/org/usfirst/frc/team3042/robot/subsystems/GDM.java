package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GDM extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Solenoid GDMRelay = new Solenoid(RobotMap.GDM_SOLENOID_ACTUATE);
	Solenoid GDMBackRelay = new Solenoid(RobotMap.GDM_BACK_SOLENOID_ACTUATE);
	boolean isActive = false;
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void actuate(){
    	
    	if(isActive){
    		GDMRelay.set(false);
    		GDMBackRelay.set(false);
    		isActive = false;
    	}
    	else{
    		GDMRelay.set(true);
    		GDMBackRelay.set(true);
    		isActive = true;
    	}
    	
    	SmartDashboard.putBoolean("GDM Out", isActive);
    }
}

