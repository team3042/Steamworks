package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.ADIS16448_IMU;
import org.usfirst.frc.team3042.lib.ADIS16448_IMU.Axis;
import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_TankDrive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.FeedbackDeviceStatus;
import com.ctre.CANTalon.MotionProfileStatus;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {
	
	public CANTalon leftMotorFront = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_FRONT);
	CANTalon leftMotorRear = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_REAR); 
	public CANTalon rightMotorFront = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_FRONT);
	CANTalon rightMotorRear = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_REAR);
	
	Solenoid gearShift = new Solenoid(RobotMap.DRIVETRAIN_SOLENOID_SHIFT);
	
	CANTalon leftEncMotor = (RobotMap.isApollo)? leftMotorFront : leftMotorRear;
    CANTalon rightEncMotor = (RobotMap.isApollo)? rightMotorRear : rightMotorFront;
    
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    
    private int leftEncoderZero, rightEncoderZero;
    private boolean leftReverseEnc = false;
    private boolean rightReverseEnc = true;
    private int leftEncSign = 1;
    private int rightEncSign = -1;
    private double scaleLeft = 1;
    private double scaleRight = 1;
    
    public boolean isHighGear = false;
   
    
    public double kPHigh = 0, kIHigh = 0, kDHigh = 0;
    public double kPLowLeft = 3, kILowLeft = 0.02, kDLowLeft = 30;
    public double kPLowRight = 4, kILowRight = 0.02, kDLowRight = 40;
    public double kPLeft = kPLowLeft, kILeft = kILowLeft, kDLeft = kDLowLeft;
    public double kPRight = kPLowRight, kIRight = kILowRight, kDRight = kDLowRight;
    public double kFLowLeft = 1.38, kFLowRight = 1.49 * 2.58 /* Wasn't Driving strait in auto_drive,
    																F-gain magic number change made it turn a less */;
    public double kFHighLeft = 0.456, kFHighRight = 0.465;
	public double kFLeft = kFLowLeft, kFRight = kFLowRight;
	double pPos = 0, iPos = 0, fPos = 0;
	int iZone = 0;
	
	private static final double WHEEL_DIAMETER_IN = 3.95; // Measured
	private static final int COUNTS_PER_REV = 360; // Quadrature counts = 1440
	
	double leftSetpoint, rightSetpoint;
	double tolerance = 4.0 / COUNTS_PER_REV;
	
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			leftMotorFront.processMotionProfileBuffer();
			rightMotorFront.processMotionProfileBuffer();
		}
	}
    
	Notifier notifier = new Notifier (new PeriodicRunnable());
	
	public DriveTrain() {
		System.out.println("Instantiating Drive Train");
		//Put the rear motors in follower mode
		leftMotorRear.changeControlMode(TalonControlMode.Follower);
    	leftMotorRear.set(leftMotorFront.getDeviceID());
    	rightMotorRear.changeControlMode(TalonControlMode.Follower);
    	rightMotorRear.set(rightMotorFront.getDeviceID());
    	
    	leftMotorFront.reverseOutput(false);
    	leftMotorFront.setInverted(false);
    	
    	rightMotorFront.setInverted(true);
    	rightMotorFront.reverseOutput(true);
    	
    	initEncoders();
    	
    	resetGyro();
		calibrateGyro();
		
		//Starting talons processing motion profile
    	leftMotorFront.changeMotionControlFramePeriod(5);
    	rightMotorFront.changeMotionControlFramePeriod(5);
    	notifier.startPeriodic(0.005);
    	
    	//Initializing PIDF
    	leftMotorFront.setProfile(1);
    	rightMotorFront.setProfile(1);
    	leftMotorFront.setPID(pPos, iPos, kDLeft);
    	rightMotorFront.setPID(pPos, iPos, kDRight);
    	leftMotorFront.setIZone(iZone);
    	rightMotorFront.setIZone(iZone);
    	leftMotorFront.setF(fPos);
    	rightMotorFront.setF(fPos);
    	
    	leftMotorFront.setProfile(0);
    	rightMotorFront.setProfile(0);
    	leftMotorFront.setPID(kPLeft, kILeft, kDLeft);
    	rightMotorFront.setPID(kPRight, kIRight, kDRight);
    	leftMotorFront.setF(kFLeft);
    	rightMotorFront.setF(kFRight);
    	
    	gearShift.set(false);
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	void initEncoders() {
		leftEncMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightEncMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		
		leftEncMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		rightEncMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
	
		leftEncMotor.configEncoderCodesPerRev(COUNTS_PER_REV);
		rightEncMotor.configEncoderCodesPerRev(COUNTS_PER_REV);
		
		leftEncMotor.reverseSensor(leftReverseEnc);
		rightEncMotor.reverseSensor(rightReverseEnc);
		
		resetEncoders();
	}
	
	public void initDefaultCommand() {
	    // Set the default command for a subsystem here.
		setDefaultCommand(new DriveTrain_TankDrive());
	}
	
	public void stop() {
		setMotorsRaw(0,0);
	}
	
	// Converts inches/second to motor percentage using feed-forward term
	public void setMotorsInchesPerSecondOpenLoop(double left, double right) {
		double leftSpeed = left / 53.4;
		double rightSpeed = 1.38 * right / 56.4;
		
		setMotors(leftSpeed, rightSpeed);
	}
	
	public void setMotors(double left, double right) {
		left = scaleLeft(left);
		right = scaleRight(right);
		
		setMotorsRaw(left, right);
	}

	public void setMotorsRaw(double left, double right) {
		left = safetyTest(left);
		right = safetyTest(right);
		
		leftMotorFront.changeControlMode(TalonControlMode.PercentVbus);
		rightMotorFront.changeControlMode(TalonControlMode.PercentVbus);
		leftMotorFront.set(left);
		rightMotorFront.set(right);		
	}
	
	private double safetyTest(double motorValue) {
	    motorValue = (motorValue < -1) ? -1 : motorValue;
	    motorValue = (motorValue > 1) ? 1 : motorValue;
	    
	    return motorValue;
	}
	
	public void shiftGear() {
		if(isHighGear) {
			shiftGearLow();
		}
		else {
			shiftGearHigh();
		}
		
		SmartDashboard.putBoolean("red = lowgear", isHighGear);
		
		leftMotorFront.setPID(kPLeft, kILeft, kDLeft);
    	rightMotorFront.setPID(kPRight, kIRight, kDRight);
    	leftMotorFront.setF(kFLeft);
    	rightMotorFront.setF(kFRight);
	}
	
	public void shiftGearLow() {
		kPLeft = kPLowLeft;
		kILeft = kILowLeft;
		kDLeft = kDLowLeft;
		
		kPRight = kPLowRight;
		kIRight = kILowRight;
		kDRight = kDLowRight;
		
		kFLeft = kFLowLeft;
		kFRight = kFLowRight;
		
		gearShift.set(false);
		isHighGear = false;
	}
	
	public void shiftGearHigh() {
		kPLeft = kPHigh;
		kILeft = kIHigh;
		kDLeft = kDHigh;
		
		kPRight = kPHigh;
		kIRight = kIHigh;
		kDRight = kDHigh;
		
		kFLeft = kFHighLeft;
		kFRight = kFHighRight;
		
		gearShift.set(true);
		isHighGear = true;
	}
	
	public void offsetPosition(double left, double right) {
		left += leftMotorFront.getPosition();
		right += rightMotorFront.getPosition();
		
		leftSetpoint = left;
		rightSetpoint = right;
		
		leftMotorFront.setProfile(1);
		rightMotorFront.setProfile(1);
		  	
		leftMotorFront.changeControlMode(TalonControlMode.Position);
		rightMotorFront.changeControlMode(TalonControlMode.Position);
		    	
		leftMotorFront.set(left);
		rightMotorFront.set(right);
	}
	
	public boolean nearSetpoint() {
		double currentLeftPosition = leftMotorFront.getPosition();
		boolean nearLeft = Math.abs(leftSetpoint - currentLeftPosition) < tolerance;
		
		double currentRightPosition = rightMotorFront.getPosition();
		boolean nearRight = Math.abs(rightSetpoint - currentRightPosition) < tolerance;
		
		return nearLeft && nearRight;
	}
	
	private double scaleLeft(double left) {
		return scaleLeft * left;
	}
	
	private double scaleRight(double right) {
		return scaleRight * right;
	}
	
	public void resetEncoders() {
		leftEncoderZero = leftEncMotor.getEncPosition();
		rightEncoderZero = rightEncMotor.getEncPosition();
	}
	
	public int getLeftEncoder() {
		return leftEncSign * (leftEncMotor.getEncPosition() - leftEncoderZero);
	}
	
	public int getRightEncoder() {
		return rightEncSign * (rightEncMotor.getEncPosition() - rightEncoderZero);
	}
	
	public double getLeftPositionInches() {
		double rotations = ((double) getLeftEncoder()) / (4 * COUNTS_PER_REV);
		
		return rotationsToInches(rotations);
	}
	
	public double getRightPositionInches() {
		double rotations = ((double) getRightEncoder()) / (4 * COUNTS_PER_REV);
		
		return rotationsToInches(rotations);
	}
	
	public double getLeftVelocity() {
		return leftEncMotor.getSpeed();
	} 
	
	public double getRightVelocity() {
		return rightEncMotor.getSpeed();
	}
	
	public double getLeftVelocityInchesPerSecond() {
		return rpmToInchesPerSecond(getLeftVelocity());
	}
	
	public double getRightVelocityInchesPerSecond() {
		return rpmToInchesPerSecond(getRightVelocity());
	}
	
	public boolean isLeftEncPresent() {
		return !(leftEncMotor.isSensorPresent(FeedbackDevice.QuadEncoder) == FeedbackDeviceStatus.FeedbackStatusPresent);
	}
	
	public boolean isRightEncPresent() {
		return !(rightEncMotor.isSensorPresent(FeedbackDevice.QuadEncoder) == FeedbackDeviceStatus.FeedbackStatusPresent);
	}
	
	public Rotation2d getGyro() {
		SmartDashboard.putData("IMU", gyro);
		
		return Rotation2d.fromDegrees((RobotMap.isApollo)? gyro.getAngleY() : gyro.getAngleX());
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	public void calibrateGyro() {
		gyro.calibrate();
	}
	
	//Motion profile functions
	public void initMotionProfile() {
		
		resetEncoders();
		
		leftMotorFront.clearMotionProfileTrajectories();
		rightMotorFront.clearMotionProfileTrajectories();
		
		leftMotorFront.setProfile(0);
		rightMotorFront.setProfile(0);
		
		leftMotorFront.changeControlMode(TalonControlMode.MotionProfile);
		rightMotorFront.changeControlMode(TalonControlMode.MotionProfile);
		leftMotorFront.set(CANTalon.SetValueMotionProfile.Disable.value);
		rightMotorFront.set(CANTalon.SetValueMotionProfile.Disable.value);
		
		leftMotorFront.clearMotionProfileHasUnderrun();
		rightMotorFront.clearMotionProfileHasUnderrun();
	}
	
	public void pushPoints(CANTalon.TrajectoryPoint leftPoint, CANTalon.TrajectoryPoint rightPoint) {
		leftMotorFront.pushMotionProfileTrajectory(leftPoint);
		rightMotorFront.pushMotionProfileTrajectory(rightPoint);
	}
	
	public MotionProfileStatus[] getMotionProfileStatus() {
		MotionProfileStatus[] motionProfileStatus = new MotionProfileStatus[2];
		motionProfileStatus[0] = new MotionProfileStatus();
		motionProfileStatus[1] = new MotionProfileStatus();
		leftMotorFront.getMotionProfileStatus(motionProfileStatus[0]);
		rightMotorFront.getMotionProfileStatus(motionProfileStatus[1]);
		
		return motionProfileStatus;
	}
	
	//Removing flag hasUnderrun if it has been logged
	public void removeUnderrunLeft() {
		leftMotorFront.clearMotionProfileHasUnderrun();
	}
	
	public void removeUnderrunRight() {
		rightMotorFront.clearMotionProfileHasUnderrun();
	}
	
	public void enableMotionProfile() {
		leftMotorFront.set(CANTalon.SetValueMotionProfile.Enable.value);
		rightMotorFront.set(CANTalon.SetValueMotionProfile.Enable.value);
	}
	
	public void holdMotionProfile() {
		leftMotorFront.set(CANTalon.SetValueMotionProfile.Hold.value);
		rightMotorFront.set(CANTalon.SetValueMotionProfile.Hold.value);
	}
	
	public void disableMotionProfile() {
		leftMotorFront.set(CANTalon.SetValueMotionProfile.Disable.value);
		rightMotorFront.set(CANTalon.SetValueMotionProfile.Disable.value);
	}
	
	private double rotationsToInches(double rotations) {
		return rotations * (Math.PI * WHEEL_DIAMETER_IN);
	}
	
	private double inchesToRotations(double inches) {
		return inches / (Math.PI * WHEEL_DIAMETER_IN);
	}
	
	private double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}
	
	private double inchesPerSecondToRPM(double inchesPerSecond) {
		return inchesToRotations(inchesPerSecond * 60);
	}
}
