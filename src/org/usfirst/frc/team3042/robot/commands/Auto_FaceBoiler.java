package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotState;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Auto_FaceBoiler extends Command {
	
	RobotState robotState;
	
    private boolean finished = false;
	
	private static final int MAX_ITERATIONS_WITHOUT_TARGET = 3;
	private double kAngleP = .6, kAngleI = 0.01, kAngleD = 4;
    private double maxCorrection = 8, correctionDeadzone = 3.5;
	private double oldGyroError = 0, sumGyroError = 0;
    private int noTargetCounter = 0;
    private double distance;
    private Rotation2d gyroGoal;

    public Auto_FaceBoiler() {
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
    	AimingParameters aim = robotState.getAimingParametersBoiler();
        
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
        
        double[] correctedSpeeds = PIDCorrectionQuadraticPlusConstant();
        
        Robot.driveTrain.setMotorsInchesPerSecondOpenLoop(correctedSpeeds[0], correctedSpeeds[1]);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
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
    
    private double[] PIDCorrectionQuadraticPlusConstant(){
    	double gyroError = gyroGoal.rotateBy(Robot.driveTrain.getGyro().inverse()).getDegrees();
    	sumGyroError += gyroError;
    	double dGyroError = gyroError - oldGyroError;
    	
    	double sign = (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError)/Math.abs((kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError));
    	
    	double correction = (sign * (kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) <= maxCorrection)? 
    			(kAngleP * gyroError + kAngleI * sumGyroError + kAngleD * dGyroError) : sign * maxCorrection;
    	
    	correction = Math.pow(correction, 2)/8.5333 + 1;
		
    	correction *= sign;
    			
    	double leftSpeed = -correction;
    	double rightSpeed = correction;
    	
    	Robot.logger.log("Gyro error: " + gyroError + ", Correction: " + correction, 3);
    	
    	oldGyroError = gyroError;
    	return new double[] {leftSpeed, rightSpeed};
    }
}
