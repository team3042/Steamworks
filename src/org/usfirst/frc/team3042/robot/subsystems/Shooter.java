package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Shooter_Stop;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem {

	private CANTalon shooterTalon = new CANTalon(RobotMap.SHOOTER_TALON);
	
	private CANTalon agitateTalon = new CANTalon(RobotMap.AGITATE_TALON);
	  
	private double kP = 0.01, kI = 0.0, kD = 0.0, kF = 0;

	public double shooterSpeed = 1000.0;
	public boolean shooterOn = false;
	public Shooter(){
		
		setPIDF();
		initEncoder();
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new Shooter_Stop());
    }
	
	public void setPIDF(){
		double P = SmartDashboard.getNumber("Shooter P", kP);
		double I = SmartDashboard.getNumber("Shooter I", kI);
		double D = SmartDashboard.getNumber("Shooter D", kD);
		double F = SmartDashboard.getNumber("Shooter F", kF);
		
		shooterTalon.setPID(P, I, D);
		shooterTalon.setF(F);
	}
	
	public void initEncoder(){
		shooterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		shooterTalon.configEncoderCodesPerRev(1024);
		shooterTalon.reverseSensor(false);
	}
	
	private void setRPM(double speed){
		shooterTalon.changeControlMode(TalonControlMode.Speed);
		
		shooterTalon.set(speed);
	}
	
	public void shoot() {
		double velocity = SmartDashboard.getNumber("Shooter speed", shooterSpeed);
		
		setRPM(SafetyTest(velocity));
		
		shooterOn = true;
	}
	
	public void stop() {
		setRPM(0);
		
		shooterOn = false;
	}
	
	public void agitate(){
		if(shooterOn == true){
			
			agitateTalon.set(.8);
			
			}else{
			agitateTalon.set(0);
		}
	}
	
	private double SafetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
}

