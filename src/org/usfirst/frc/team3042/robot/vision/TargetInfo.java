package org.usfirst.frc.team3042.robot.vision;

/**
 * Used for storing information on a detected target
 *
 */
public class TargetInfo {
	protected double x, y, distance;
	
	public TargetInfo(double x, double y, double distance) {
        this.x = x;
        this.y = y;
        this.distance = distance;
    }
	
	public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
    
    public double getDistance() {
    	return distance;
    }
}
