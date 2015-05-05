package robot.shapes;

import org.opencv.core.Point;

public class Square {
	private Point center;
	private Point lowPt;
	
	public Square(Point center, Double halfHeight){
		this.center = center;
		this.lowPt = new Point(center.x,center.y-halfHeight);
	}

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
	
}
