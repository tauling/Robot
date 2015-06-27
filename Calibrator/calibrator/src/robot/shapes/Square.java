package robot.shapes;

import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

public class Square extends RotatedRect implements Comparable<Square> {
	// private Point upPt;
	// private Size size;
	// private Point lowPt;
	// private Point lowerLeftEdge;
	// private Point upperRightEdge;
	// private Double halfWidth;
	// private Double halfHeight;
	private int colorID;

	public Square(RotatedRect rect, int colorID) {
		super(rect.center, rect.size, rect.angle);
		this.colorID = colorID;
	}

	public Square(Point center, Size size, double angle, int colorID) {
		super(center, size, angle);
		this.colorID = colorID;
	}

	public Point getHighPt() {
		Point[] pts = new Point[4];
		super.points(pts);

		Point minPoint = new Point(10000, 10000);
		Point minPoint2 = new Point(10000, 10000);

		for (Point pt : pts) {
			if (minPoint.y > pt.y) {
				minPoint2 = minPoint;
				minPoint = pt;
			} else {
				if (minPoint2.y > pt.y) {
					minPoint2 = pt;
				}
			}
		}

		return new Point((minPoint.x + minPoint2.x) / 2.0,
				(minPoint.y + minPoint2.y) / 2.0);
	}

	public Point getLowPt() {
		Point[] pts = new Point[4];
		super.points(pts);

		Point maxPoint = new Point(0, 0);
		Point maxPoint2 = new Point(0, 0);

		for (Point pt : pts) {
			if (maxPoint.y < pt.y) {
				maxPoint2 = maxPoint;
				maxPoint = pt;
			} else {
				if (maxPoint2.y < pt.y) {
					maxPoint2 = pt;
				}
			}
		}

		return new Point((maxPoint.x + maxPoint2.x) / 2.0,
				(maxPoint.y + maxPoint2.y) / 2.0);
	}

	// public Square(Point center, Double halfHeight, Point lowerEdgeLeft,
	// int colorID) {
	// this.colorID = colorID;
	// this.center = center;
	// this.lowerLeftEdge = lowerEdgeLeft;
	// this.halfWidth = center.x - lowerEdgeLeft.x;
	// this.halfHeight = lowerEdgeLeft.y - center.y;
	// this.lowPt = new Point(center.x, center.y + this.halfHeight);
	// this.upperRightEdge = new Point(lowPt.x + this.halfWidth, center.y
	// - this.halfHeight);
	// }

	// public Square(Point center, Point lowerEdgeLeft, int colorID) {
	// this.colorID = colorID;
	// this.center = center;
	// this.lowPt = new Point(center.x, lowerEdgeLeft.y);
	// this.lowerLeftEdge = lowerEdgeLeft;
	// this.halfWidth = center.x - lowerEdgeLeft.x;
	// this.halfHeight = lowerEdgeLeft.y - center.y;
	// this.upperRightEdge = new Point(lowPt.x + this.halfWidth, center.y
	// - this.halfHeight);
	// }

	@Override
	public String toString() {
		Point[] pts = new Point[4];
		super.points(pts);
		return "square, center: {" + (int) this.center.x + ","
				+ (int) this.center.y + "} points: " + pts[0] + ", " + pts[1]
				+ ", " + pts[2] + ", " + pts[3] + "; angle: " + angle
				+ " and colorid: " + getColorID();
	}

	// public Point getLowerLeftEdge() {
	// return lowerLeftEdge;
	// }
	//
	// public Point getUpperRightEdge() {
	// return upperRightEdge;
	// }
	//
	// public Double getHalfWidth() {
	// return halfWidth;
	// }
	//
	// public Double getHalfHeight() {
	// return halfHeight;
	// }

	public int getColorID() {
		return colorID;
	}

	public int compareTo(Square otherSquare) {
		if (this.center.y < otherSquare.center.y) {
			return -1;
		} else if (this.center.y > otherSquare.center.y) {
			return 1;
		} else {
			return 0;
		}
	}

}
