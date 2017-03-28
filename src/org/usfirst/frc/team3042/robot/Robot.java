
package org.usfirst.frc.team3042.robot;

import org.spectrum3847.RIOdroid.RIOdroid;
import org.usfirst.frc.team3042.robot.commands.AutoMode_DoNothing;
import org.usfirst.frc.team3042.robot.commands.AutoMode_DriveForward;
import org.usfirst.frc.team3042.robot.commands.AutoMode_GearCenter;
import org.usfirst.frc.team3042.robot.commands.AutoMode_GearLeft;
import org.usfirst.frc.team3042.robot.commands.AutoMode_GearRight;
import org.usfirst.frc.team3042.robot.commands.AutoMode_ShootLeft;
import org.usfirst.frc.team3042.robot.commands.AutoMode_ShootRight;
import org.usfirst.frc.team3042.robot.vision.AimingParameters;
import org.usfirst.frc.team3042.robot.vision.TestUpdateReceiver;
import org.usfirst.frc.team3042.robot.vision.VisionServer;
import org.usfirst.frc.team3042.robot.subsystems.Climber;
import org.usfirst.frc.team3042.robot.subsystems.CompressorSubsystem;
import org.usfirst.frc.team3042.robot.subsystems.DriveTrain;
import org.usfirst.frc.team3042.robot.subsystems.GDM;
import org.usfirst.frc.team3042.robot.subsystems.Intake;
import org.usfirst.frc.team3042.robot.subsystems.LEDSwitch;
import org.usfirst.frc.team3042.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.hal.PDPJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static OI oi;
	public static Logger logger;
	public static VisionServer visionServer;
	public static RobotState robotState;
	public static TestUpdateReceiver testUpdateReceiver;
	public static final CompressorSubsystem compressor = new CompressorSubsystem();
	public static final DriveTrain driveTrain = new DriveTrain();
	public static final Climber climber = new Climber();
	public static final Shooter shooter = new Shooter();
	public static final Intake intake = new Intake();
	public static final GDM gdm = new GDM();
	public static final LEDSwitch ledSwitch = new LEDSwitch();
	
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	Command autonomousCommand;
    SendableChooser<Command> autonomousChooser;
    public static FileIO fileIO = new FileIO();
    private int LOGGER_LEVEL = 5;
    boolean useConsole = true, useFile = false;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
        logger = new Logger(useConsole, useFile, LOGGER_LEVEL);
        
        RIOdroid.initUSB();
        
        if (RobotMap.isPhone) {
	        robotState = RobotState.getInstance();
	        visionServer = VisionServer.getInstance();
	        testUpdateReceiver = new TestUpdateReceiver();
	        //visionServer.addVisionUpdateReceiver(testUpdateReceiver);
	        visionServer.addVisionUpdateReceiver(robotState);
        }
        
		autonomousChooser = new SendableChooser<Command>();
        autonomousChooser.addDefault("Default (Do Nothing)", new AutoMode_DoNothing());
        autonomousChooser.addObject("Drive Forward", new AutoMode_DriveForward());
        autonomousChooser.addObject("Gear Center", new AutoMode_GearCenter());
        autonomousChooser.addObject("Gear Left", new AutoMode_GearLeft());
        autonomousChooser.addObject("Gear Right", new AutoMode_GearRight());
        autonomousChooser.addObject("Shoot Left", new AutoMode_ShootLeft());
        autonomousChooser.addObject("Shoot Right", new AutoMode_ShootRight());
        
        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
        
        SmartDashboard.putNumber("Shooter speed", Shooter.shooterSpeed);
        SmartDashboard.putNumber("Shooter P", Shooter.kP);
        SmartDashboard.putNumber("Shooter D", Shooter.kD);
        SmartDashboard.putNumber("Shooter I", Shooter.kI);
        
        SmartDashboard.putBoolean("red = lowgear", driveTrain.isHighGear);
    }
     
    public void disabledInit(){
    	Robot.logger.log("Disabled Init", 1);
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above, adding additional choosers,
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
    	Robot.logger.log("Autonomous Init", 1);
    	
    	autonomousCommand = (Command) autonomousChooser.getSelected();
    	
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
    	Robot.logger.log("Teleop Init", 1);
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this (below) line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        Robot.driveTrain.shiftGearLow();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();  
        //Here is where you would put smart dashboard outputs, to put a number on the smart dashbard follow this format
        //SmartDashboard.putNumber("example number", number or variable here);
        
        SmartDashboard.putNumber("PDP Amp Draw 14", pdp.getCurrent(14));
        //System.out.println(pdp.getCurrent(14));
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
}

  