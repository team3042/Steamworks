package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Climber_Stop;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    CANTalon climber = new CANTalon(RobotMap.CLIMBER_TALON);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Climber_Stop());
    }
    
    public void SetPower(double motorValue){
    	motorValue = SafetyTest(motorValue);
    	climber.set(motorValue);
    }
    
    private double SafetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
}

