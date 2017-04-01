package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_HopperShootLeft extends CommandGroup {
	
	//Commands that need to be canceled when the command group ends
	Command spinup = new Shooter_SpinUp(),
			shoot  = new Shooter_Shoot();

    public AutoMode_HopperShootLeft() {
    	
    	//addSequential(new DriveTrain_ShiftGears());
    	
    	//Drive to the hopper that is to the right of the robot
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			false));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, 4.5, 5));
    	addSequential(new Auto_Drive(AutoType.TURN_LEFT, 8.0, 5, 28));
    	
    	//Pause for balls to fill
    	addSequential(new Auto_WaitForFillup());

    	//Spin up the shooter wheel
    	addParallel(spinup);
    	
    	//Drive to the shooting position
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_BackFromLeft.getLeftTrajectory(),
    			AutoTrajectory_BackFromLeft.getRightTrajectory(),
    			true));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, -1, -5));
    	addSequential(new Auto_Drive(AutoType.TURN_LEFT, -6.2, -5, 6));
    	
    	//Fire the agitator to feed the shooter
    	addParallel(shoot);
    	
    	//Use the camera to improve aim
    	
    }
    /*
    protected void end() {
    	gracefulEnd();
    }
    
    protected void interrupted() {
    	gracefulEnd();
    }
    
    protected void gracefulEnd() {
    	spinup.cancel();
    	shoot.cancel();
    }
    */
}
