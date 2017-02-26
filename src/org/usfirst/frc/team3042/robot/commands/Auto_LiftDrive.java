package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotState;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Auto_LiftDrive extends Command {
    
    RobotState robotState;
    
    private boolean finished = false;
    
    private static final double MIN_VIEW_DISTANCE = 36;
    private static final int MAX_ITERATIONS_WITHOUT_TARGET = 1;
    private boolean useVision = true;
    private double distance, oldEncoderDistance;
    private Rotation2d gyroGoal;
    //private double kDistanceP, kDistanceI, kDistanceD;
    private double kAngleP = .3, kAngleI = 0, kAngleD = 3;
    private double maxCorrection = 8, correctionDeadzone = 3.5;
    private double oldGyroError = 0, sumGyroError = 0;
    private int noTargetCounter = 0;
    
    // Parameters for logistic function
    private static final double DISTANCE_OFFSET = 17; // Inches
    private static final double MAX_SPEED = -24; // Inches/Second
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
        
        useVision = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double encoderDistance = -Robot.driveTrain.getLeftPositionInches();
        
        // Calculating the current goals based on vision if far enough away, otherwise using other sensor data
        if (useVision) {
            AimingParameters aim = robotState.getAimingParameters();
            
            if (!aim.isValid()) {
                Robot.logger.log("No target in view!", 2);
                
                noTargetCounter++;
                if (noTargetCounter > MAX_ITERATIONS_WITHOUT_TARGET) {
                	finished = true;
                }
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
    	
		//Robot.logger.log(((useVision)? "using vision" : "vision disabled") + " at distance: " + distance, 3);
        
        double speed = speedFromDistance(distance);
        
        double[] correctedSpeeds = PIDCorrectionQuadraticPlusConstant(speed);
        
        Robot.driveTrain.setMotorsInchesPerSecondOpenLoop(correctedSpeeds[0], correctedSpeeds[1]);
        
        SmartDashboard.putNumber("Left Drivetrain FPS", Robot.driveTrain.getLeftVelocityInchesPerSecond() / 12);
		SmartDashboard.putNumber("Right Drivetrain FPS", Robot.driveTrain.getRightVelocityInchesPerSecond() / 12);
        
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
        
        double speed = MAX_SPEED / (1 + Math.pow(Math.E, -STEEPNESS * (distance - X_OFFSET)));
        
        return speed;
    }
    
    private double[] PIDCorrectionLogisticCurve(double speed){
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double sign = (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError)/Math.abs((kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError));
    	
    	double correction = (sign * (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) <= maxCorrection)? 
    			(kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) : sign * maxCorrection;
    			
    	correction = sign * 20/(1 + 20*Math.pow(Math.E, -.55 * Math.abs(correction)));		
    	
    	Robot.logger.log("Correction: " + correction, 3);
    	
    	double leftSpeed = speed - correction;
    	double rightSpeed = speed + correction;
        
    	oldGyroError = gyroError;
        return new double[] {leftSpeed, rightSpeed};
    }
    
    private double[] PIDCorrectionDeadzone(double speed) {
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double sign = (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError)/Math.abs((kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError));
    	
    	double correction = (sign * (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) <= maxCorrection)? 
    			(kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) : sign * maxCorrection;
    			
    	if (Math.abs(gyroError) < correctionDeadzone) {
    		correction = 0;
    	}
    			
		Robot.logger.log("Correction: " + correction, 3);
    	
    	double leftSpeed = speed - correction;
    	double rightSpeed = speed + correction;
        
    	oldGyroError = gyroError;
        return new double[] {leftSpeed, rightSpeed};
    }
    
    private double[] PIDCorrectionQuadraticPlusConstant(double speed){
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double sign = (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError)/Math.abs((kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError));
    	
    	double correction = (sign * (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) <= maxCorrection)? 
    			(kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) : sign * maxCorrection;
    	
    	correction = Math.pow(correction, 2)/8.5333 + 1;
		
    	correction *= (useVision)? sign : -sign;
    			
    	double leftSpeed = speed - correction;
    	double rightSpeed = speed + correction;
    	
    	Robot.logger.log("Gyro error: " + gyroError + ", Correction: " + correction, 3);
    	
    	oldGyroError = gyroError;
    	return new double[] {leftSpeed, rightSpeed};
    }
}
