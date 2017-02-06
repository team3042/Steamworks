package org.usfirst.frc.team3042.robot.vision.messages;

public class TargetTypeMessage extends VisionMessage {
	
	private static final String BOILER_TARGET = "boiler";
	private static final String LIFT_TARGET = "lift";
	
	private String message;
	
	private TargetTypeMessage(String message) {
		this.message = message;
	}
	
	public static TargetTypeMessage getBoilerMessage() {
		return new TargetTypeMessage(BOILER_TARGET);
	}
	
	public static TargetTypeMessage getLiftMessage() {
		return new TargetTypeMessage(LIFT_TARGET);
	}

	@Override
	public String getType() {
		return "targetType";
	}

	@Override
	public String getMessage() {
		return message;
	}

}
