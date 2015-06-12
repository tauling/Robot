package robot.shapes;

import org.opencv.core.Point;

public class Circle extends Shape {

	private double radius;

	public Circle(Point center, Double radius) {
		this.center = center;
		this.lowPt = new Point(center.x, center.y + radius);
		this.radius = radius;
	}

	@Override
	public String toString() {
		return "circle, center: " + this.center + " lowest point: "
				+ this.lowPt + " radius: " + this.radius;
	}

	public double getRadius() {
		return radius;
	}

}
