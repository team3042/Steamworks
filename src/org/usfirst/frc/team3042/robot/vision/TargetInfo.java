package org.usfirst.frc.team3042.robot.vision;

/**
 * Used for storing information on a detected target
 *
 */
public class TargetInfo {
	protected double x, y;
	
	public TargetInfo(double x, double y) {
        this.x = x;
        this.y = y;
    }
	
	public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
