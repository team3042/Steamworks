package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.commands.LEDSwitch_SetOn;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LEDSwitch extends Subsystem {

    Solenoid LEDSolenoid = new Solenoid(0);
    Solenoid LEDSolenoid2 = new Solenoid(2);
        
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new LEDSwitch_SetOn());
    }
    
    public void setOn(){
        LEDSolenoid.set(true);
        LEDSolenoid2.set(true);
    }
    
    public void setOff(){
        LEDSolenoid.set(false);
        LEDSolenoid2.set(false);
    }
}

