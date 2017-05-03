package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_ShootLeft extends CommandGroup {
	public AutoMode_ShootLeft(){
		addParallel(new Shooter_SpinUp(false));
		addSequential(new Auto_WaitForSpinup());
		addParallel(new Shooter_Shoot());
		addSequential(new Auto_WaitForShootTenBalls());
		addParallel(new Shooter_Stop());
		
		addSequential(new Auto_Drive(AutoType.TURN_LEFT, 6.54, 3, 5));
		addSequential(new Auto_Drive(AutoType.STRAIGHT, 6, 5));
	}
}
