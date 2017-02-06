package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotState;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Auto_LiftDrive extends Command {
    
    RobotState robotState;
    
    private boolean finished = false;
    
    private static final double MIN_VIEW_DISTANCE = 1.5;
    private boolean useVision = true;
    private double distance, oldEncoderDistance;
    private Rotation2d gyroGoal;
    //private double kDistanceP, kDistanceI, kDistanceD;
    private double kAngleP = 0, kAngleI = 0, kAngleD = 0;
    private double oldGyroError = 0, sumGyroError = 0;
    
    // Parameters for logistic function
    private static final double DISTANCE_OFFSET = 8; // Inches
    private static final double MAX_SPEED = 36; // Inches/Second
    private static final double STEEPNESS = 1/3;
    private static final double X_OFFSET = 15;

    public Auto_LiftDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
        
        robotState = RobotState.getInstance();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.logger.log("Initialize", 1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double encoderDistance = Robot.driveTrain.getLeftPositionInches();
        
        // Calculating the current goals based on vision if far enough away, otherwise using other sensor data
        if (useVision) {
            AimingParameters aim = robotState.getAimingParameters();
            
            if (!aim.isValid()) {
                Robot.logger.log("No target in view!", 2);
                finished = true; // TODO: Maybe only terminate after n loops without target view
                return;
            }
            
            distance = aim.getDistance();
            Rotation2d angleOffset = aim.getAngle();
            gyroGoal = Robot.driveTrain.getGyro().rotateBy(angleOffset);
            
            // If we are close enough, stop using vision as we will lose the target soon and transition to gyro driving
            if (distance < MIN_VIEW_DISTANCE) {
                useVision = false;
            }
        } else {
            // Using encoders, determine remaining distance    
            distance -= (encoderDistance - oldEncoderDistance);
        }
        double speed = speedFromDistance(distance);
        
        double[] correctedSpeeds = PIDCorrection(speed);
        
        Robot.driveTrain.setMotorsInchesPerSecondOpenLoop(correctedSpeeds[0], correctedSpeeds[1]);
        
        oldEncoderDistance = encoderDistance;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (finished || distance < DISTANCE_OFFSET);
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
    
    // Using logistic function to convert distance to speed
    private double speedFromDistance(double distance) {
    	// How far we want to be from wall, placing on peg 11" out
        distance -= DISTANCE_OFFSET;
        
        double speed = MAX_SPEED / (1 - Math.pow(Math.E, -STEEPNESS * (distance - X_OFFSET)));
        
        return speed;
    }
    
    private double[] PIDCorrection(double speed){
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double leftSpeed = speed - (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError);
    	double rightSpeed = speed + (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError);
        
    	oldGyroError = gyroError;
        return new double[] {leftSpeed, rightSpeed};
    }
}
