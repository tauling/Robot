package test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Point;

import robot.shapes.Square;

public class Test {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Point center = new Point(0, 0);
		Point center2 = new Point(0, 100);
		Point center3 = new Point(0, -100);

		Point lowPt = new Point(0, 0);
		Point lowPt2 = new Point(0, 100);
		Point lowPt3 = new Point(0, -100);

		Point lowerEdgeLeft = new Point(0, 0);
		Point lowerEdgeLeft2 = new Point(0, 100);
		Point lowerEdgeLeft3 = new Point(0, -100);

		List<Square> squareList = new ArrayList<Square>();
		Square s1 = new Square(center, lowPt, lowerEdgeLeft, 1);
		Square s2 = new Square(center2, lowPt2, lowerEdgeLeft2, 1);
		Square s3 = new Square(center3, lowPt3, lowerEdgeLeft3, 1);
		squareList.add(s1);
		squareList.add(s2);
		squareList.add(s3);

		System.out.println(squareList);
		Collections.sort(squareList);
		System.out.println(squareList);
	}
}
