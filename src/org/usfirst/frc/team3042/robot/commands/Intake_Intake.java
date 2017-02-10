package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 */
public class Intake_Intake extends Command {


    public Intake_Intake() {
      requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	
    	Robot.intake.intake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        SmartDashboard.putNumber("Intake measured speed", Robot.intake.getSpeed());
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
    	Robot.logger.log("Interrupted", 1);
    	
    }
}
