package robot.shapes;

import org.opencv.core.Point;

public class Circle {
	private Point center;
	private Point lowPt; 
	
	public Circle(Point center, Double radius){
		this.center = center;
		this.lowPt = new Point(center.x,center.y-radius);
	}

	public Point getCenter() {
		return center;
	}

	public void setCenter(Point center) {
		this.center = center;
	}

	public Point getLowPt() {
		return lowPt;
	}

	public void setLowPt(Point lowPt) {
		this.lowPt = lowPt;
	}

}
