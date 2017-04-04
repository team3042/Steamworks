package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter_SpinUp extends Command {

	Timer time = new Timer();
	double timeLimit = 1.0;
	boolean PIDStarted, isAuto;
	
    public Shooter_SpinUp(boolean isAuto) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	this.isAuto = isAuto;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	Robot.shooter.setPIDF();
    	PIDStarted = false;
    	time.reset();
    	time.start();
    	Robot.shooter.spinup();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Shooter RPM", Robot.shooter.getRPM());
    	
    	if ((time.get() >= timeLimit) && (!PIDStarted)) {
    		Robot.shooter.spin(isAuto);
    		PIDStarted = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.logger.log("End", 1);
    	shutDown();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.logger.log("Interrupted", 1);
    	shutDown();
    }

    protected void shutDown() {
    	Robot.shooter.spindown();
    	time.stop();
    }
}
