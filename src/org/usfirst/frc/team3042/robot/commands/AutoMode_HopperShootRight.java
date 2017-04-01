package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

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
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			false));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, 4.7, 5));
    	addSequential(new Auto_Drive(AutoType.TURN_RIGHT, 12.0, 5, 21));
    	//Pause for balls to fill
    	addSequential(new Auto_WaitForFillup());

    	//Spin up the shooter wheel
    	addParallel(spinup);
    	
    	//Drive to the shooting position
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_BackFromRight.getLeftTrajectory(),
    			AutoTrajectory_BackFromRight.getRightTrajectory(),
    			true));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, -1, -5));
    	addSequential(new Auto_Drive(AutoType.TURN_RIGHT, -10, -5, 15));
    	
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
