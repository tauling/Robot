package robot.navigate;

import org.opencv.core.Point;

public class Ball {
	private Integer id;
	private Point pos;
	private Point lowPt;
	public static int counter = 0;
	
	public Ball(Point pos, Point lowPt){
		this.id = counter++;
		this.pos = pos;
		this.lowPt = lowPt;
	}
	
	public Integer getId() {
		return id;
	}

	public Point getPos() {
		return pos;
	}
	
	public Point getLowPt(){
		return lowPt;
	}
	
}
