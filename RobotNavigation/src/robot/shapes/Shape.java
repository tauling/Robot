package robot.shapes;

import org.opencv.core.Point;

public abstract class Shape {
	protected Point lowPt;
	protected Point center;

	public Point getLowPt() {
		return lowPt;
	}

	public void setLowPt(Point lowPt) {
		this.lowPt = lowPt;
	}

	public Point getCenter() {
		return center;
	}

	public void setCenter(Point center) {
		this.center = center;
	}
	
	public abstract String toString();

}
