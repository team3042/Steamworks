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
    
    private static final double MIN_VIEW_DISTANCE = 1.5;
    private boolean useVision = true;
    private double distance, oldEncoderDistance;
    private Rotation2d angleOffset;
    private double kAngleKP, kAngleKI, kAngleD, kDistanceP, kDistanceI, kDistanceD;

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
        
        if (useVision) {
            AimingParameters aim = robotState.getAimingParameters();
            
            if (!aim.isValid()) {
                Robot.logger.log("No target in view!", 2);
                return;
            }
            
            distance = aim.getDistance();
            angleOffset = aim.getAngle();
            
            // If we are close enough, stop using vision as we will lose the target soon and transition to gyro driving
            if(distance < MIN_VIEW_DISTANCE) {
                useVision = false;
            }
        } else {
            // Using encoders, determine remaining distance    
            distance -= (encoderDistance - oldEncoderDistance);
            angleOffset = Rotation2d.fromDegrees(0);
        }
        double speed = speedFromDistance(distance);
        
        speed = PIDCorrection(speed);
        
        oldEncoderDistance = encoderDistance;
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
    
    // Using logistic function to convert distance to speed
    private double speedFromDistance(double distance) {
        double speed = 0;
        
        return speed;
    }
    
    private double PIDCorrection(double speed){
        
        return speed;
    }
}
