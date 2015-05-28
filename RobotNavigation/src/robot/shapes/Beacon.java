package robot.shapes;

import org.opencv.core.Point;

public class Beacon extends Square {
	
	private int upperColorID;

	public Beacon(Point center, Double halfHeight, Point lowerEdgeLeft,
			int lowerColorID, int upperColorID) {
		super(center, halfHeight, lowerEdgeLeft, lowerColorID);
		this.upperColorID = upperColorID;
	}
	
	public Beacon(Point center, Point lowerEdgeLeft,
			int lowerColorID, int upperColorID) {
		super(center, lowerEdgeLeft, lowerColorID);
		this.upperColorID = upperColorID;
		System.out.println("My Beaconcolor: " + lowerColorID + ", " + upperColorID + "; first one should be " + super.getColorID() + " and color comb: " + (upperColorID*10 + super.getColorID()));
	}
	
	public int getColorComb() {
		return upperColorID*10 + super.getColorID();
	}

	
}
