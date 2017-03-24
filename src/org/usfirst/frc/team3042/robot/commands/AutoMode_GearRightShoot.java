package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_GearRightShoot extends CommandGroup {
	public AutoMode_GearRightShoot(){
		addSequential(new Auto_Drive(AutoType.TURN_RIGHT,2,0.1));
		//addSequential(new Shooter_Shoot(),1);
		//addSequential(new Shooter_Stop());
	}
}
