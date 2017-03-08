package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.commands.Auto_Drive.AutoType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMode_GearLeft extends CommandGroup {

    public AutoMode_GearLeft() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new Auto_Drive(AutoType.STRAIGHT, -7.5, -3));
    	addSequential(new Auto_Drive(AutoType.TURN_RIGHT, 1.55, 2, 0));
    	//addSequential(new Vision_TrackLift());
    	addSequential(new LEDSwitch_SetOn());
    	addSequential(new Auto_LiftDrive());
    	addSequential(new LEDSwitch_SetOff());
    }
}
