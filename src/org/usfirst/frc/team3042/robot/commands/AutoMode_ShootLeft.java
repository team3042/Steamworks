package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_ShootLeft extends CommandGroup {
	public AutoMode_ShootLeft(){
		addSequential(new Auto_Drive(AutoType.TURN_LEFT,2,4,0));
		//addSequential(new Shooter_Shoot(),1);
		//addSequential(new Shooter_Stop());
	}
}
