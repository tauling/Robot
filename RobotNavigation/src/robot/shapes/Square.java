package robot.shapes;

import org.opencv.core.Point;

public class Square extends Shape {
	private Point lowerLeftEdge;
	private Point upperRightEdge;
	private int colorID;

	public Square(Point center, Double halfHeight, Point lowerEdgeLeft, int colorID) {
		this.colorID = colorID;
		this.center = center;
		this.lowPt = new Point(center.x, center.y - halfHeight);
		this.lowerLeftEdge = lowerEdgeLeft;
		Double halfWidth = center.x-lowerEdgeLeft.x;
		this.upperRightEdge = new Point(lowerEdgeLeft.x+2*halfWidth,lowerEdgeLeft.y+2*halfHeight);
	}
	
	public Square(Point center, Point lowPt, Point lowerEdgeLeft) {
		this.center = center;
		this.lowPt = lowPt;
		this.lowerLeftEdge = lowerEdgeLeft;
		Double halfWidth = center.x-lowerEdgeLeft.x;
		Double halfHeight = center.y -lowPt.y;
		this.upperRightEdge = new Point(lowerEdgeLeft.x+2*halfWidth,lowerEdgeLeft.y+2*halfHeight);
	}

	@Override
	public String toString() {
		return "square, center: " + this.center + " lowest point:" + this.lowPt;
	}

	public Point getLowerLeftEdge() {
		return lowerLeftEdge;
	}

	public Point getUpperRightEdge() {
		return upperRightEdge;
	}

	public int getColorID() {
		return colorID;
	}

}
