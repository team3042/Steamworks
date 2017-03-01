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
    Compressor compressor = new Compressor(RobotMap.COMPRESSOR_PORT);
   
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CompressorSubsystem() {
    	System.out.println("Instantiating compressor");
        compressor.start();
        
        compressor.setClosedLoopControl(true);
        
        System.out.println("Finished instantiating compressor");
    }
}

