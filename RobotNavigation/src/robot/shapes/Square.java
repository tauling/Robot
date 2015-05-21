package robot.shapes;

import org.opencv.core.Point;

public class Square extends Shape implements Comparable{
	private Point lowerLeftEdge;
	private Point upperRightEdge;
	private Double halfWidth;
	private Double halfHeight;
	private int colorID;

	public Square(Point center, Double halfHeight, Point lowerEdgeLeft, int colorID) {
		this.colorID = colorID;
		this.center = center;
		this.lowerLeftEdge = lowerEdgeLeft;
		this.halfWidth = center.x-lowerEdgeLeft.x;
		this.halfHeight = lowerEdgeLeft.y-center.y;
		this.lowPt = new Point(center.x, center.y + this.halfHeight);
		this.upperRightEdge = new Point(lowerEdgeLeft.x+2*halfWidth,center.y-this.halfHeight);
	}
	
	public Square(Point center, Point lowPt, Point lowerEdgeLeft, int colorID) {
		this.colorID = colorID;
		this.center = center;
		this.lowPt = lowPt;
		this.lowerLeftEdge = lowerEdgeLeft;
		this.halfWidth = center.x-lowerEdgeLeft.x;
		this.halfHeight = lowerEdgeLeft.y-center.y;
		this.upperRightEdge = new Point(lowerEdgeLeft.x+2*halfWidth,center.y-halfHeight);
	}

	@Override
	public String toString() {
		return "square, center: {" + (int)this.center.x+","+ (int)this.center.y+ "} lowest point:" + this.lowPt;
	}

	public Point getLowerLeftEdge() {
		return lowerLeftEdge;
	}

	public Point getUpperRightEdge() {
		return upperRightEdge;
	}

	public Double getHalfWidth() {
		return halfWidth;
	}

	public Double getHalfHeight() {
		return halfHeight;
	}
	
	public int getColorID() {
		return colorID;
	}

	public int compareTo(Square otherSquare) {
		 if (this.getCenter().y<otherSquare.getCenter().y){
	            return -1;
	        }else{
	            return 1;
	        }
	}

	@Override
	public int compareTo(Object another) {
		// TODO Auto-generated method stub
		return 0;
	}

}
