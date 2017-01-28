package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.ADIS16448_IMU;
import org.usfirst.frc.team3042.lib.Rotation2d;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.DriveTrain_TankDrive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.FeedbackDeviceStatus;
import com.ctre.CANTalon.MotionProfileStatus;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	CANTalon leftMotorFront = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_FRONT);
	//CANTalon leftMotorRear = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_REAR); 
	CANTalon rightMotorFront = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_FRONT);
	//CANTalon rightMotorRear = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_REAR);
	
	Relay gearShiftRight = new Relay(RobotMap.DRIVETRAIN_SOLENOID_RIGHT);
	Relay gearShiftLeft = new Relay(RobotMap.DRIVETRAIN_SOLENOID_LEFT);
	
	CANTalon leftEncMotor = leftMotorFront;
    CANTalon rightEncMotor = rightMotorFront;
    
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    
    private int leftEncoderZero, rightEncoderZero;
    public int enCounts;
    private boolean leftReverseEnc;
    private boolean rightReverseEnc;
    private int leftEncSign;
    private int rightEncSign;
    
    private boolean isHighGear = false;
   
    
    public double kP = 0, kI = 0, kD = 0;
	public double kF;
	double pPos = 0, iPos = 0, fPos = 0;
	int iZone = 0;
	
	private static final double WHEEL_DIAMETER_IN = 4.0;
	private static final double COUNTS_PER_REV = 1024; //TODO: Determine actual value
	
	double leftSetpoint, rightSetpoint;
	//double tolerance = enCounts;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			leftMotorFront.processMotionProfileBuffer();
			rightMotorFront.processMotionProfileBuffer();
		}
	}
    
	Notifier notifier = new Notifier (new PeriodicRunnable());
	
	public DriveTrain() {
		//Put the rear motors in follower mode
		//leftMotorRear.changeControlMode(TalonControlMode.Follower);
    	//leftMotorRear.set(leftMotorFront.getDeviceID());
    	//rightMotorRear.changeControlMode(TalonControlMode.Follower);
    	//rightMotorRear.set(rightMotorFront.getDeviceID());
    	
    	leftMotorFront.reverseOutput(false);
    	leftMotorFront.setInverted(false);
    	
    	rightMotorFront.setInverted(true);
    	rightMotorFront.reverseOutput(true);
    	
    	initEncoders();
    	
		calibrateGyro();
		
		//Starting talons processing motion profile
    	leftMotorFront.changeMotionControlFramePeriod(5);
    	rightMotorFront.changeMotionControlFramePeriod(5);
    	notifier.startPeriodic(0.005);
    	
    	//Initializing PIDF
    	leftMotorFront.setProfile(1);
    	rightMotorFront.setProfile(1);
    	leftMotorFront.setPID(pPos, iPos, kD);
    	rightMotorFront.setPID(pPos, iPos, kD);
    	leftMotorFront.setIZone(iZone);
    	rightMotorFront.setIZone(iZone);
    	leftMotorFront.setF(fPos);
    	rightMotorFront.setF(fPos);
    	
    	leftMotorFront.setProfile(0);
    	rightMotorFront.setProfile(0);
    	leftMotorFront.setPID(kP, kI, kD);
    	rightMotorFront.setPID(kP, kI, kD);
    	leftMotorFront.setF(kF);
    	rightMotorFront.setF(kF);
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	void initEncoders() {
		leftEncMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightEncMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		
		leftEncMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
		rightEncMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
	
		leftEncMotor.configEncoderCodesPerRev(enCounts);
		rightEncMotor.configEncoderCodesPerRev(enCounts);
		
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
	
	public void shiftGear(){
		if(isHighGear){
			gearShiftRight.set(Relay.Value.kOn);
			gearShiftLeft.set(Relay.Value.kOn);
			isHighGear = false;
		}
		else{
			gearShiftRight.set(Relay.Value.kOff);
			gearShiftLeft.set(Relay.Value.kOff);
			isHighGear = true;
		}
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
	/*
	public boolean nearSetpoint() {
		double currentLeftPosition = leftMotorFront.getPosition();
		boolean nearLeft = Math.abs(leftSetpoint - currentLeftPosition) < tolerance;
		
		double currentRightPosition = rightMotorFront.getPosition();
		boolean nearRight = Math.abs(rightSetpoint - currentRightPosition) < tolerance;
		
		return nearLeft && nearRight;
	}
	*/
	private double scaleLeft(double left) {
		return left;
	}
	
	private double scaleRight(double right) {
		return right;
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
		double rotations = getLeftEncoder() / COUNTS_PER_REV;
		
		return rotationsToInches(rotations);
	}
	
	public double getRightPositionInches() {
		double rotations = getRightEncoder() / COUNTS_PER_REV;
		
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
		return Rotation2d.fromDegrees(gyro.getAngle());
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	public void calibrateGyro() {
		gyro.calibrate();
	}
	
	//Motion profile functions
	public void initMotionProfile() {
		
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
	
	private double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}
}
