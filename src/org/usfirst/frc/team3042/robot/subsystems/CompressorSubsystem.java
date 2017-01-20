package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CompressorSubsystem extends Subsystem {

	// Put methods for controlling this subsystem
    // here. Call these from Commands.
    Compressor compressor = new Compressor(0);
   
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CompressorSubsystem() {
        compressor.start();
        
        compressor.setClosedLoopControl(true);
    }
}
