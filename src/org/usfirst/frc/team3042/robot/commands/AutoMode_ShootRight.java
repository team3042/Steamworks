package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_ShootRight extends CommandGroup {
	public AutoMode_ShootRight(){
		addSequential(new Auto_Drive(AutoType.TURN_RIGHT,2,4,0));
	}
}
