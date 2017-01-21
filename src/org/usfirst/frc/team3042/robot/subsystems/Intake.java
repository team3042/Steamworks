package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Intake_Stop;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @param <SafetyTest>
 *
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
    CANTalon intake = new CANTalon(RobotMap.INTAKE_TALON);
    
    private double intakeSpeed = 0.8;
    private double exhaustSpeed = -0.8;
    
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Intake_Stop());
	}
	
	private void setPower(double motorValue){
		motorValue = SafetyTest(motorValue);
		intake.set(motorValue);
	}
	
	public void stop() {
    	setPower(0);
    }
	
    public void intake() {
    	setPower(intakeSpeed);
    }
    
    public void exhaust() {
    	setPower(exhaustSpeed);
    }
	
	 private double SafetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
    
}

