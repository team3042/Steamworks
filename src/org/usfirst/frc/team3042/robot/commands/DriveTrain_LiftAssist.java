package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotState;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTrain_LiftAssist extends Command {
    
    RobotState robotState;
    
    private boolean finished = false;

	private final double deadzone = 0.1;
    
    final int LEFT = 0, RIGHT = 1;
    Timer time = new Timer();
    double[] oldTime = new double[] {0, 0};
    double[] currentPower = new double[] {0,0};
    double maxAccel = 3.6; //motor power per second
    
    private static final double MIN_VIEW_DISTANCE = 2.2;
    private static final int MAX_ITERATIONS_WITHOUT_TARGET = 1;
    private boolean useVision = true;
    private double distance, oldEncoderDistance;
    private Rotation2d gyroGoal;
    //private double kDistanceP, kDistanceI, kDistanceD;
    private double kAngleP = 0.1, kAngleI = 0, kAngleD = 0.3;
    private double maxCorrection = 1;
    private double oldGyroError = 0, sumGyroError = 0;
    private int noTargetCounter = 0;
    
    // Parameters for logistic function
    private static final double DISTANCE_OFFSET = 6; // Inches
    private static final double MAX_SPEED = 36; // Inches/Second
    private static final double STEEPNESS = 1/3;
    private static final double X_OFFSET = 15;

    public DriveTrain_LiftAssist() {
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
        
        double speed = Robot.oi.joystickLeft.getY();
        
        speed = (Math.abs(speed) < deadzone)? 0 : speed;
        
        //speed = restrictAccel(speed, LEFT);
        speed *= MAX_SPEED;
        
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
    
    private double[] PIDCorrection(double speed){
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double correction = ((kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) <= maxCorrection)? 
    			(kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) : maxCorrection;
    	
    	double leftSpeed = speed - correction;
    	double rightSpeed = speed + correction;
        
    	oldGyroError = gyroError;
        return new double[] {leftSpeed, rightSpeed};
    }
    
    private double restrictAccel (double goalValue, int SIDE) {
		double currentTime = time.get();
	    double dt = currentTime - oldTime[SIDE];
	    oldTime[SIDE] = currentTime;
	        
	    double maxDSpeed = maxAccel * dt;
        maxDSpeed *= (goalValue >= currentPower[SIDE])? 1 : -1;
         
        currentPower[SIDE] = (Math.abs(maxDSpeed) > Math.abs(goalValue - currentPower[SIDE]))? 
                goalValue : maxDSpeed + currentPower[SIDE];
        return currentPower[SIDE];
	}
}

