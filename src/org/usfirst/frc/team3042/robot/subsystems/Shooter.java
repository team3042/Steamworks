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
	private CANTalon agitatorTalon = new CANTalon(RobotMap.AGITATE_TALON);
	  
	private double kP = 0.01, kI = 0.0, kD = 0.0, kF = 0;

	public double shooterSpeed = 1000.0, agitatorSpeed = 0.8;
	
	public Shooter() {
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
	
	private void setShooterRPM(double speed){
		shooterTalon.changeControlMode(TalonControlMode.Speed);
		
		shooterTalon.set(speed);
	}
	
	private void setShooterRaw(double speed) {
		shooterTalon.changeControlMode(TalonControlMode.PercentVbus);
		
		shooterTalon.set(0);
	}
	
	private void spinAgitator(double speed) {
		agitatorTalon.set(safetyTest(speed));
	}
	
	public void shoot() {
		double velocity = SmartDashboard.getNumber("Shooter speed", shooterSpeed);
		
		setShooterRPM(velocity);
		spinAgitator(agitatorSpeed);
	}
	
	public void stop() {
		setShooterRaw(0);
		spinAgitator(0);
	}
	
	private double safetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
}

