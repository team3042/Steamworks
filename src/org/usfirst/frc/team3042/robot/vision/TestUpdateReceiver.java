package org.usfirst.frc.team3042.robot.vision;

import java.util.List;

import org.usfirst.frc.team3042.robot.Robot;

public class TestUpdateReceiver implements VisionUpdateReceiver {

	@Override
	public void gotUpdate(VisionUpdate update) {
		List<TargetInfo> targets = update.getTargets();
		
		Robot.logger.log("Vision update received (Time: " + update.getCapturedAgoMs() + ")", 2);
		for(int i = 0; i < targets.size(); i++) {
			Robot.logger.log("\tTarget recieved (x: " + targets.get(i).getX() + ", y: " + targets.get(i).getY() + ")", 3);
		}
	}
}
