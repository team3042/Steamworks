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
	  
	double P = 0.01, I = 0.0, D = 0.0;

	public double shooterSpeed = 1000.0;
	
	public Shooter(){
		setPID();
		InitEncoder();
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new Shooter_Stop());
    }
	
	public void setPID(){
		double P = SmartDashboard.getNumber("Shooter P", 0.01);
		double I = SmartDashboard.getNumber("Shooter I", 0);
		double D = SmartDashboard.getNumber("Shooter D", 0);
		
		shooterTalon.setPID(P, I, D);	
	}
	
	public void InitEncoder(){
		shooterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		shooterTalon.configEncoderCodesPerRev(1024);
		shooterTalon.reverseSensor(false);
	}
	
	public void SetRPM(double speed){
		shooterTalon.changeControlMode(TalonControlMode.Speed);
		
		shooterTalon.set(speed);
	}
	
	
}

