package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.MotionProfileStatus;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 *
 */
public class Auto_Drive extends Command {
	
	double leftDistance, rightDistance, leftMaxSpeed, rightMaxSpeed;
	
	//Current point
	int pointNumber = 0;

	//Time between each point in ms
	int itp = 30;
		
	//Time for each filter in ms
	double time1 = 400, time2 = 200;
	
	double wheelbaseWidth = 28.25; // Inches
	
	//AutoType enum found at bottom of class
	AutoType autoType;
	
	AutoTrajectory_MotionProfile motionProfileLeft;
	AutoTrajectory_MotionProfile motionProfileRight;
	
	MotionProfileStatus[] status;
	
	CANTalon.TrajectoryPoint[] leftTrajectory;
	CANTalon.TrajectoryPoint[] rightTrajectory;
	
    public Auto_Drive(AutoType autoType, double distance, double maxSpeed, double radius) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	
    	this.autoType = autoType;
    	
    	if(autoType == AutoType.STRAIGHT) {
    		leftDistance = distance;
    		rightDistance = distance;
    		
    		leftMaxSpeed= maxSpeed;
    		rightMaxSpeed = maxSpeed;
    	}
    	else if(autoType == AutoType.TURN_LEFT) {
    		double leftRadius = radius - wheelbaseWidth / 2;
    		double rightRadius = radius + wheelbaseWidth / 2;
    		
    		double leftScale, rightScale;
    		//Creating scale for each side in relation to center
    		if(radius != 0) {
    			leftScale = leftRadius / radius;
    			rightScale = rightRadius / radius;
    		}
    		else {
    			leftScale = leftRadius;
    			rightScale = rightRadius;
    		}
    		
    		double maxScale = Math.max(leftScale, rightScale);
    		leftScale /= maxScale;
    		rightScale /= maxScale;
    		
    		leftDistance = leftScale * distance;
    		rightDistance = rightScale * distance;
    		leftMaxSpeed = leftScale * maxSpeed;
    		rightMaxSpeed = rightScale * maxSpeed;
    	}
    	else if(autoType == AutoType.TURN_RIGHT) {
    		double leftRadius = radius + wheelbaseWidth / 2;
    		double rightRadius = radius - wheelbaseWidth / 2;
    		
    		double leftScale, rightScale;
    		//Creating scale for each side in relation to center
    		if(radius != 0) {
    			leftScale = leftRadius / radius;
    			rightScale = rightRadius / radius;
    		}
    		else {
    			leftScale = leftRadius;
    			rightScale = rightRadius;
    		}
    		
    		double maxScale = Math.max(leftScale, rightScale);
    		leftScale /= maxScale;
    		rightScale /= maxScale;
    		
    		leftDistance = leftScale * distance;
    		rightDistance = rightScale * distance;
    		leftMaxSpeed = leftScale * maxSpeed;
    		rightMaxSpeed = rightScale * maxSpeed;
    	}
    }
    
    public Auto_Drive(AutoType autoType, double distance, double maxSpeed) {
    	this(autoType, distance, maxSpeed, 0.0);
    }

    

	// Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	
    	Robot.driveTrain.initMotionProfile();
    	
    	motionProfileLeft = new AutoTrajectory_MotionProfile(itp, time1, time2, leftMaxSpeed, leftDistance);
    	motionProfileRight = new AutoTrajectory_MotionProfile(itp, time1, time2, rightMaxSpeed, rightDistance);
    	
    	System.out.println("Left: " + leftDistance + ", Right: " + rightDistance);
    	
    	leftTrajectory = motionProfileLeft.calculateProfile();
    	rightTrajectory = motionProfileRight.calculateProfile();
    	
    	for(int i = 0; i < leftTrajectory.length; i++) {
    		if(i < rightTrajectory.length) {
    			Robot.driveTrain.pushPoints(leftTrajectory[i], rightTrajectory[i]);
    			System.out.println("Left Distance: " + leftTrajectory[i].position + "Right Distance: " + rightTrajectory[i].position +", " + i);
    		}
    		else {
    			CANTalon.TrajectoryPoint zeroPoint = new CANTalon.TrajectoryPoint();
    			zeroPoint.isLastPoint = true;
    			zeroPoint.position = rightTrajectory[i - 1].position;
    			zeroPoint.timeDurMs = itp;
    			zeroPoint.velocity = 0;
    			zeroPoint.zeroPos = false;
    			Robot.driveTrain.pushPoints(leftTrajectory[i], zeroPoint);
    		}
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//actually does the driving along path generated by AutoTrajectory_MotionProfile,
    	//keeping track of status in order to avoid errors
    	status = Robot.driveTrain.getMotionProfileStatus();
    	
    	if(status[0].btmBufferCnt > 5) {
    		Robot.driveTrain.enableMotionProfile();
    	}
    	
    	if(status[0].hasUnderrun) {
    		Robot.logger.log("Left Underrun", 2);
    		Robot.driveTrain.removeUnderrunLeft();
    	}
    	if(status[1].hasUnderrun) {
    		Robot.logger.log("Right Underrun", 2);
    		Robot.driveTrain.removeUnderrunRight();
    	}
    	
    	Robot.logger.log("Left Encoder Actual: " + Robot.driveTrain.getLeftEncoder() + "\tLeft Encoder Goal: " + status[0].activePoint.position * 1440 +
    			"\tLeft Goal Speed: " + status[0].activePoint.velocity + "\tLeft Actual Speed: " + Robot.driveTrain.getLeftVelocity(), 1);
    	Robot.logger.log("Right Encoder Actual: " + Robot.driveTrain.getRightEncoder() + "\tRight Encoder Goal: " + status[1].activePoint.position * 1440 +
    			"\tRight Goal Speed: " + status[1].activePoint.velocity + "\tRight Actual Speed: " + Robot.driveTrain.getRightVelocity() + "\n", 1);
    	
    	
    	SmartDashboard.putNumber("Left Drivetrain Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Drivetrain Encoder", Robot.driveTrain.getRightEncoder());
		SmartDashboard.putNumber("Left Drivetrain RPM", Robot.driveTrain.getLeftVelocity());
		SmartDashboard.putNumber("Right Drivetrain RPM", Robot.driveTrain.getRightVelocity());
		SmartDashboard.putNumber("Left Drivetrain FPS", Robot.driveTrain.getLeftVelocityInchesPerSecond() / 12);
		SmartDashboard.putNumber("Right Drivetrain FPS", Robot.driveTrain.getRightVelocityInchesPerSecond() / 12);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return status[1].activePoint.isLastPoint;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.logger.log("End", 1);
    	
    	Robot.driveTrain.disableMotionProfile();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.logger.log("Interrupt", 1);
    	
    	Robot.driveTrain.disableMotionProfile();
    }
    
    public enum AutoType {
    	STRAIGHT, TURN_LEFT, TURN_RIGHT;
    }
}
