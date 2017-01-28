package org.usfirst.frc.team3042.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3042.lib.InterpolatingDouble;
import org.usfirst.frc.team3042.lib.InterpolatingTreeMap;
import org.usfirst.frc.team3042.lib.RigidTransform2d;
import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.lib.Translation2d;
import org.usfirst.frc.team3042.robot.vision.TargetInfo;
import org.usfirst.frc.team3042.robot.vision.TargetTracker;
import org.usfirst.frc.team3042.robot.vision.VisionUpdate;
import org.usfirst.frc.team3042.robot.vision.VisionUpdateReceiver;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

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
	private TargetTracker targetTracker;
	private VisionUpdate mostRecentUpdate = null;
	private double oldLeftPosition = 0;
	private double oldRightPosition = 0;
	
	private static final int MAX_OBSERVATIONS = 50;
	private static final double OBSERVATION_INTERVAL = 0.020;
	private static boolean running = true;
	
	private static final int CAMERA_X_OFFSET = 0;
	private static final int CAMERA_Y_OFFSET = 0;
	private static final Translation2d ROBOT_TO_CAMERA = new Translation2d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET);
	
	private RobotState() {
		reset(0, new RigidTransform2d());
		
		notifier = new Notifier(runnable);
		notifier.startPeriodic(OBSERVATION_INTERVAL);
	}
	
	public void reset(double startTime, RigidTransform2d fieldToRobotInitial) {
		robotPose = new InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d>(MAX_OBSERVATIONS);
		robotPose.put(new InterpolatingDouble(startTime), fieldToRobotInitial);
		
		robotVelocity = new RigidTransform2d.Delta(0, 0, 0);
		
		targetTracker = new TargetTracker();
	}
	
	public RigidTransform2d getRobotPose(double timestamp) {
		return robotPose.get(new InterpolatingDouble(timestamp));
	}
	
	// TODO: Add velocity calculation for prediction of future pose if needed for vision accuracy
	// Adding another entry to the robotPose map with encoder and gyro values from the drivetrain
	private void updatePose() {
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
	private void updateVision() {
		List<Translation2d> fieldToTargets = new ArrayList<>();
		
		if (!(mostRecentUpdate == null || mostRecentUpdate.getTargets().isEmpty())) {
			double timestamp = mostRecentUpdate.getCapturedAtTimestamp();
			List<TargetInfo> targets = mostRecentUpdate.getTargets();
			
			RigidTransform2d fieldToCamera = robotPose.get(new InterpolatingDouble(timestamp))
					.transformBy(RigidTransform2d.fromTranslation(ROBOT_TO_CAMERA));
			
			// TODO: Check sign of angles sent from phone
			for (TargetInfo target : targets) {
				double cameraToTargetX = target.getDistance() * Math.cos(target.getX());
				double cameraToTargetY = target.getDistance() * Math.sin(target.getX());
				
				Translation2d cameraToTarget = new Translation2d(cameraToTargetX, cameraToTargetY);
				
				fieldToTargets.add(fieldToCamera.transformBy(RigidTransform2d.fromTranslation(cameraToTarget)).getTranslation());
			}
			
			targetTracker.update(timestamp, fieldToTargets);
		}
	}

	@Override
	public void gotUpdate(VisionUpdate update) {
		mostRecentUpdate = update;	
	}

}
