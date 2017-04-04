package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Shooter_Stop;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem {

	private CANTalon shooterTalon = new CANTalon(RobotMap.SHOOTER_TALON);
	private CANTalon agitatorTalon = new CANTalon(RobotMap.AGITATE_TALON);
	  
	public static double kP = 0.34, kI = 0.0, kD = 3.4, kF = .0271;

	public static double shooterSpeed = -2900, agitatorSpeed = 0.90;
	
	public static double shooterSpeedHopperBlue = -2950;
	
	public double maxSpeedError = 500;
	
	private int shooterTalonZero = 0;
	
	public Shooter() {
		System.out.println("Instantiating shooter");
		initEncoder();
		
		shooterTalon.enableBrakeMode(false);
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new Shooter_Stop());
    }
	
	public void setPIDF(){
	
		double P = SmartDashboard.getNumber("Shooter P", kP);
		double I = SmartDashboard.getNumber("Shooter I", kI);
		double D = SmartDashboard.getNumber("Shooter D", kD);
		shooterTalon.setPID(P, I, D);
		
		//shooterTalon.setPID(kP, kI, kD);
		shooterTalon.setF(kF);
	}
	
	public void initEncoder(){
		shooterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		shooterTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooterTalon.configEncoderCodesPerRev(1024);
		shooterTalon.reverseSensor(false);
		
		shooterTalonZero = shooterTalon.getEncPosition();
	}
	
	public double getRPM() {
		return shooterTalon.getSpeed();
	}
	
	private void setShooterRPM(double speed){
		shooterTalon.changeControlMode(TalonControlMode.Speed);
		
		shooterTalon.set(speed);
	}
	
	private void setShooterRaw(double speed) {
		shooterTalon.changeControlMode(TalonControlMode.PercentVbus);
		
		shooterTalon.set(safetyTest(speed));
	}
	
	private void spinAgitator(double speed) {
		agitatorTalon.set(safetyTest(speed));
	}
	
	public void spin(boolean isAuto) {
		double velocity = (isAuto)? shooterSpeedHopperBlue : shooterSpeed; //SmartDashboard.getNumber("Shooter speed", shooterSpeed);
		setShooterRPM(velocity);	
	}
	
	public void spinup() {
		shooterTalon.changeControlMode(TalonControlMode.PercentVbus);
		shooterTalon.set(-1.0);
	}
	
	public void spindown() {
		setShooterRaw(0);
	}
	
	public void shoot() {
		//spinup();
		

       /* Robot.logger.log("Spinning agitator with RPM Error: " + Math.abs(velocity - getRPM()), 3);
		
		if (Math.abs(velocity - getRPM()) < maxSpeedError) {
		    spinAgitator(agitatorSpeed);
		} else {
		    spinAgitator(0);
		}
		*/
		spinAgitator(agitatorSpeed);
	}
	
	public void stop() {
		//setShooterRaw(0);
		spinAgitator(0);
	}
	
	private double safetyTest(double motorValue){
    	motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
	
	public int getEncoderVal() {
		return shooterTalon.getEncPosition() - shooterTalonZero;
	}
}

