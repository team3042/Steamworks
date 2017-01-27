package org.usfirst.frc.team3042.robot.vision;

import java.util.List;

public class TestUpdateReceiver implements VisionUpdateReceiver {

	@Override
	public void gotUpdate(VisionUpdate update) {
		List<TargetInfo> targets = update.getTargets();
		
		System.out.println("Vision update received (Time: " + update.getCapturedAgoMs() + ")");
		for(int i = 0; i < targets.size(); i++) {
			System.out.println("\tTarget recieved (x: " + targets.get(i).getX() + ", y: " + targets.get(i).getY() + 
					", distance: " + targets.get(i).getDistance() + ")");
		}
	}
}
