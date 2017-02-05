package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain_Calibrate extends Command {

    public DriveTrain_Calibrate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	
    	Robot.driveTrain.setMotorsRaw(1, 1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putNumber("Left Drivetrain Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Drivetrain Encoder", Robot.driveTrain.getRightEncoder());
		SmartDashboard.putNumber("Left Drivetrain RPM", Robot.driveTrain.getLeftVelocity());
		SmartDashboard.putNumber("Right Drivetrain RPM", Robot.driveTrain.getRightVelocity());
		SmartDashboard.putNumber("Gyro Value", Robot.driveTrain.getGyro().getDegrees());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.logger.log("End", 1);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.logger.log("Interrupt", 1);
    }
}
