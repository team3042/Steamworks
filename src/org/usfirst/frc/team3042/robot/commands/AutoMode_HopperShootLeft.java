package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_HopperShootLeft extends CommandGroup {

    public AutoMode_HopperShootLeft() {
    	
    	//addSequential(new DriveTrain_ShiftGears());
    	
    	//Drive to the hopper that is to the left of the robot
    	addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_HopperToRight.getLeftTrajectory(),
    			AutoTrajectory_HopperToRight.getRightTrajectory(),
    			true));
    	
    	//addSequential(new Auto_Drive(AutoType.STRAIGHT, 4.5, 5));
    	//addSequential(new Auto_Drive(AutoType.TURN_LEFT, 8.0, 5, 28));
    	
    	//Pause for balls to fill
    	addSequential(new Auto_WaitForFillup());

    	//Spin up the shooter wheel
    	addParallel(new Shooter_SpinUp(true));
    	
    	//Drive to the shooting position
    	/*addSequential(new Auto_FollowTrajectory(
    			AutoTrajectory_BackFromLeft.getLeftTrajectory(),
    			AutoTrajectory_BackFromLeft.getRightTrajectory(),
    			true));
    	*/
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, -1, -5));
    	addSequential(new Auto_Drive(AutoType.TURN_LEFT, -2.92, -5, 2));
    	
    	//Fire the agitator to feed the shooter
    	addParallel(new Shooter_Shoot());
    	
    	//Use the camera to improve aim
    	
    }
}
