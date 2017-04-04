package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_HopperShootRight extends CommandGroup {
	
    public AutoMode_HopperShootRight() {

    	//Drive to the hopper that is to the right of the robot
    	addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			false));
    	
    	//addSequential(new Auto_Drive(AutoType.STRAIGHT, 4.5, 5));
    	//addSequential(new Auto_Drive(AutoType.TURN_RIGHT, 8.0, 5, 28));
    	
    	//Pause for balls to fill
    	addSequential(new Auto_WaitForFillup());

    	//Spin up the shooter wheel
    	addParallel(new Shooter_SpinUp(true));
    	
    	//Drive to the shooting position
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_BackFromRight.getLeftTrajectory(),
    			AutoTrajectory_BackFromRight.getRightTrajectory(),
    			true));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, -1, -5));
    	addSequential(new Auto_Drive(AutoType.TURN_RIGHT, -2.35, -5, 2));
    	
    	//Fire the agitator to feed the shooter
    	addParallel(new Shooter_Shoot());
    	
    	//Use the camera to improve aim
    	
    }
}
