package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.InterpolatingDouble;
import org.usfirst.frc.team3042.lib.InterpolatingTreeMap;
import org.usfirst.frc.team3042.lib.RigidTransform2d;
import org.usfirst.frc.team3042.lib.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class RobotState implements Runnable {
	private static RobotState instance = null;
	
	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
	
	private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> robotPose;
	private RigidTransform2d.Delta robotVelocity;
	private double oldLeftPosition = 0;
	private double oldRightPosition = 0;
	
	private static int maxObservations = 100;
	private static int observationIntervalMs = 10;
	private static boolean running = true;
	
	private RobotState() {
		reset(0, new RigidTransform2d());
		
		new Thread(this).start();
	}
	
	public void reset(double startTime, RigidTransform2d fieldToRobotInitial) {
		robotPose = new InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d>(maxObservations);
		robotPose.put(new InterpolatingDouble(startTime), fieldToRobotInitial);
		
		robotVelocity = new RigidTransform2d.Delta(0, 0, 0);
	}
	
	public RigidTransform2d getRobotPose(double timestamp) {
		return robotPose.get(new InterpolatingDouble(timestamp));
	}

	@Override
	public void run() {
		while(running) {
			updateState();
			
			try {
				Thread.sleep(observationIntervalMs);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
	}
	
	// TODO: Add velocity calculation for prediction of future pose if needed for vision accuracy
	// Adding another entry to the robotPose map with encoder and gyro values from the drivetrain
	private void updateState() {
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

}
