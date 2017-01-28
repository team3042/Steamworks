package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Intake_Stop;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @param <SafetyTest>
 *
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
    CANTalon intake = new CANTalon(RobotMap.INTAKE_TALON);
    
    private double kP = 0.05, kI = 0.00005, kD = 1.0, kF = .1;
    
    private double intakeSpeed = -1500;
    private double exhaustSpeed = 1000;
    
    private double intakeZero = 0;
    
    public Intake(){
    	initEncoder();
    }
    
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Intake_Stop());
	}
	
	public void setPIDF(){
		double P = SmartDashboard.getNumber("Intake P", kP);
		double I = SmartDashboard.getNumber("Intake I", kI);
		double D = SmartDashboard.getNumber("Intake D", kD);
		
		intake.setPID(kP, kI, kD);
		intake.setF(kF);
	}
	
	public void initEncoder(){
		intake.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		intake.configEncoderCodesPerRev(1024);
		intake.reverseSensor(false);
		
		intakeZero = intake.getEncPosition();
	}
	
	private void setPower(double motorValue){
		motorValue = safetyTest(motorValue);
		intake.set(motorValue);
	}
	
	private void setRPM(double speed){
		intake.changeControlMode(CANTalon.TalonControlMode.Speed);
		
		intake.set(speed);
	}
	
	public void stop() {
    	setPower(0);
    }
	
    public void intake() {
        setPIDF();
        
        double speed = SmartDashboard.getNumber("Intake speed", intakeSpeed);
        
    	setRPM(intakeSpeed);
    }
    
    public void exhaust() {
    	setRPM(exhaustSpeed);
    }
	
	 private double safetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
	 
	public double getSpeed() {
	    return intake.getSpeed();
	}
    
}

