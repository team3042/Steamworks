package org.usfirst.frc.team3042.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3042.lib.InterpolatingDouble;
import org.usfirst.frc.team3042.lib.InterpolatingTreeMap;
import org.usfirst.frc.team3042.lib.RigidTransform2d;
import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.lib.Translation2d;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;
import org.usfirst.frc.team3042.robot.vision.TargetInfo;
import org.usfirst.frc.team3042.robot.vision.TargetTrack;
import org.usfirst.frc.team3042.robot.vision.TargetTracker;
import org.usfirst.frc.team3042.robot.vision.TargetTracker.TrackReport;
import org.usfirst.frc.team3042.robot.vision.VisionUpdate;
import org.usfirst.frc.team3042.robot.vision.VisionUpdateReceiver;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState implements VisionUpdateReceiver {
	private static RobotState instance = null;
	
	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
	
	private final Notifier notifier;
	private final Runnable runnable = new Runnable() {
		@Override
		public void run() {
			if (running) {
				updatePose();
				updateVision();
			}
		}
	};
	
	private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> robotPose;
	private RigidTransform2d.Delta robotVelocity;
	private TargetTrack targetTrack;
	private VisionUpdate mostRecentUpdate = null;
	private double oldLeftPosition = 0;
	private double oldRightPosition = 0;
	
	private static final int MAX_OBSERVATIONS = 50;
	private static final double OBSERVATION_INTERVAL = 0.020;
	private static boolean running = true;
	
	private static boolean newVisionUpdate = false;
	
	private static final int CAMERA_X_OFFSET = 12;
	private static final int CAMERA_Y_OFFSET = 8;
	private static final Translation2d ROBOT_TO_CAMERA = new Translation2d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET);
	
	private static int currentTrackId = -1;
	private static final double CURRENT_TARGET_WEIGHT = 1.5;
	private static final double RECENT_WEIGHT = 1.0;
	private static final double STABILITY_WEIGHT = 2.0;
	
	
	
	private RobotState() {
		reset(0, new RigidTransform2d());
		
		notifier = new Notifier(runnable);
		notifier.startPeriodic(OBSERVATION_INTERVAL);
	}
	
	public void reset(double startTime, RigidTransform2d fieldToRobotInitial) {
		robotPose = new InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d>(MAX_OBSERVATIONS);
		robotPose.put(new InterpolatingDouble(startTime), fieldToRobotInitial);
		
		robotVelocity = new RigidTransform2d.Delta(0, 0, 0);
		
		targetTrack = null;
	}
	
	public RigidTransform2d getRobotPose(double timestamp) {
		return robotPose.get(new InterpolatingDouble(timestamp));
	}
	
	// TODO: Add velocity calculation for prediction of future pose if needed for vision accuracy
	// Adding another entry to the robotPose map with encoder and gyro values from the drivetrain
	private synchronized void updatePose() {
		double leftPosition = Robot.driveTrain.getLeftPositionInches();
		double rightPosition = Robot.driveTrain.getRightPositionInches();
		double dLeftPosition = leftPosition - oldLeftPosition;
		double dRightPosition = rightPosition - oldRightPosition;
		
		Rotation2d currentHeading = Robot.driveTrain.getGyro();
		
		InterpolatingDouble timestamp = new InterpolatingDouble(Timer.getFPGATimestamp());
		RigidTransform2d lastPose = robotPose.lastEntry().getValue();
		
		// Calculating the current robot pose using the change in gyro and encoder readings, and putting it in the map
		RigidTransform2d.Delta velocityWithGyro = new RigidTransform2d.Delta(
				(dLeftPosition + dRightPosition) / 2, 0, lastPose.getRotation().inverse().rotateBy(currentHeading).getRadians());
		RigidTransform2d currentPose = lastPose.transformBy(RigidTransform2d.fromVelocity(velocityWithGyro));
		robotPose.put(timestamp, currentPose);
		
		oldLeftPosition = leftPosition;
		oldRightPosition = rightPosition;
	}
	
	// Taking the most recent vision update and storing it in relation to the field using the robot pose when the image was taken
	private synchronized void updateVision() {
			//targetTracker.update(timestamp, fieldToTargets);
	    Translation2d fieldToTarget = null;
	    
	    if (!(mostRecentUpdate == null || mostRecentUpdate.getTargets().isEmpty()) && newVisionUpdate){
	        double timestamp = mostRecentUpdate.getCapturedAtTimestamp();
	        TargetInfo target = mostRecentUpdate.getTargets().get(0);
	        
	        RigidTransform2d fieldToCamera = robotPose.getInterpolated(new InterpolatingDouble(timestamp))
                    .transformBy(RigidTransform2d.fromTranslation(ROBOT_TO_CAMERA));
	        
	        // TODO: Check sign of angles sent from phone
	        double cameraToTargetX = target.getDistance() * Math.cos(target.getX());
            double cameraToTargetY = target.getDistance() * Math.sin(target.getX());
            
            Translation2d cameraToTarget = new Translation2d(cameraToTargetX, cameraToTargetY);
	        
	        fieldToTarget = fieldToCamera.transformBy(RigidTransform2d.fromTranslation(cameraToTarget)).getTranslation();
            
            Robot.logger.log("Target at X: " + fieldToTarget.getX() + ", Y: " + fieldToTarget.getY() + "\n", 3);
	        
	        targetTrack = new TargetTrack(timestamp, fieldToTarget,0);
	    }
	}
	
	// Looks through the list of targets and determines the best based on weights, returning aiming parameters
	public synchronized AimingParameters getAimingParameters() {
	    if (targetTrack == null) {
            return new AimingParameters(Rotation2d.fromDegrees(0), 0, false);
        }
	    
	    TrackReport track = new TrackReport(targetTrack);
	    
	    Translation2d fieldToTarget = track.fieldToTarget;
	    RigidTransform2d fieldToRobot = getRobotPose(Timer.getFPGATimestamp());
	    
	    RigidTransform2d robotToTarget = fieldToRobot.inverse().transformBy(RigidTransform2d.fromTranslation(fieldToTarget));
	    
	    double distance = robotToTarget.getTranslation().norm();
	    Rotation2d angle = robotToTarget.getRotation();
	    
	    return new AimingParameters(angle, distance, true);
	}

	@Override
	public void gotUpdate(VisionUpdate update) {
		newVisionUpdate = true;
		
		mostRecentUpdate = update;
	}
	
	public void outputToSmartDashboard() {
		RigidTransform2d pose = robotPose.lastEntry().getValue();
		
		SmartDashboard.putNumber("Robot X", pose.getTranslation().getX());
		SmartDashboard.putNumber("Robot Y", pose.getTranslation().getY());
		SmartDashboard.putNumber("Robot Theta", pose.getRotation().getDegrees());
		
		if(targetTrack != null){
    		TrackReport track = new TrackReport(targetTrack);
    		
    		SmartDashboard.putNumber("Goal X", track.fieldToTarget.getX());
    		SmartDashboard.putNumber("Goal Y", track.fieldToTarget.getY());
		}
	}

}
