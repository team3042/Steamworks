package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_HopperShootRight extends CommandGroup {
	
	//Commands that need to be canceled when the command group ends
	Command spinup = new Shooter_SpinUp(),
			shoot  = new Shooter_Shoot();

    public AutoMode_HopperShootRight() {

    	//Drive to the hopper that is to the right of the robot
    	addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			false));
    	
    	//Pause for balls to fill
    	addSequential(new Auto_WaitForFillup());

    	//Spin up the shooter wheel
    	addParallel(spinup);
    	
    	//Drive to the shooting position
    	addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_BackFromRight.getLeftTrajectory(),
    			AutoTrajectory_BackFromRight.getRightTrajectory(),
    			true));
    	
    	//Fire the agitator to feed the shooter
    	addParallel(new Shooter_Shoot());
    	
    	//Use the camera to improve aim
    	
    }
    
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
}
