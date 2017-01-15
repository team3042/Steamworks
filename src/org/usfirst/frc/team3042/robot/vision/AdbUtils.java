package org.usfirst.frc.team3042.robot.vision;

import org.spectrum3847.RIOdroid.RIOdroid;


//Code from spectrum 3847 (Look up riodroid on github)
public class AdbUtils{
	
	public static String adbReverseForward(int remotePort, int localPort) {
		try {
			return RIOdroid.executeCommand("reverse tcp:" + remotePort + " tcp:" + localPort);
		} catch(Exception e) {
			return e.getMessage();	
		}
	}
}
