package robot.shapes;

import org.opencv.core.Point;

public class Circle extends Shape{
	
	public Circle(Point center, Double radius){
		this.center = center;
		this.lowPt = new Point(center.x,center.y-radius);
	}

	@Override
	public String toString() {
		return "circle, center: "+this.center+" lowest point:"+this.lowPt;
	}
	
}
