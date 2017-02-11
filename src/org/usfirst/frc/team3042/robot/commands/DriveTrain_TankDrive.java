package org.usfirst.frc.team3042.robot.commands;
import org.usfirst.frc.team3042.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain_TankDrive extends Command {
	private final double speedScale = 1.0;
	private final double deadzone = 0.1;
	
	//Inertia dampening
    final int LEFT = 0, RIGHT = 1;
    Timer time = new Timer();
    double[] oldTime = new double[] {0, 0};
    double[] currentPower = new double[] {0,0};
    double maxAccel = 3.6; //motor power per second
    
    public DriveTrain_TankDrive() {
    	requires(Robot.driveTrain);
    }

	protected void initialize() {
		Robot.logger.log("Initialize",1);
		Robot.driveTrain.setMotors(0, 0);
		time.start();
	}

	protected void execute() {
		SmartDashboard.putNumber("Left Drivetrain Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Drivetrain Encoder", Robot.driveTrain.getRightEncoder());
		SmartDashboard.putNumber("Left Drivetrain RPM", Robot.driveTrain.getLeftVelocity());
		SmartDashboard.putNumber("Right Drivetrain RPM", Robot.driveTrain.getRightVelocity());
		SmartDashboard.putNumber("Left Drivetrain FPS", Robot.driveTrain.getLeftVelocityInchesPerSecond() / 12);
		SmartDashboard.putNumber("Right Drivetrain FPS", Robot.driveTrain.getRightVelocityInchesPerSecond() / 12);
		SmartDashboard.putNumber("Gyro Value", Robot.driveTrain.getGyro().getDegrees());
		
		double leftPower = -Robot.oi.joystickLeft.getY() * speedScale;
		double rightPower = -Robot.oi.joystickRight.getY() * speedScale;
		
		leftPower = (Math.abs(leftPower) < deadzone)? 0 : leftPower;
		rightPower = (Math.abs(rightPower) < deadzone)? 0 : rightPower;
		
		leftPower = restrictAccel(leftPower, LEFT);
        rightPower = restrictAccel(rightPower, RIGHT);
        
        Robot.driveTrain.setMotors(leftPower,rightPower);
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		Robot.logger.log("End",1);
	}

	protected void interrupted() {
		Robot.logger.log("Interrupted",1);
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
