package robot.opencv;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


import android.util.Log;

import robot.shapes.Ball;
import robot.shapes.Beacon;
import robot.shapes.Circle;
import robot.shapes.Square;

public class ImageProcessor {
	
	// TODO Check if offsets for homography are needed
	
	private static double mMinContourArea = 0.1; // Minimum contour area in percent for contours filtering

	private String TAG; // Tag for log-messages sent to logcat
	
	/**
	 * Constructor method.
	 * 
	 * @param TAG Used for messages to logcat.
	 */
	public ImageProcessor(String TAG) {
		this.TAG = TAG;
	}
	
	/**
	 * Finds contours of the objects within the given grayImage.
	 * @param grayImage input image
	 * @return a list of contours
	 */
	public List<MatOfPoint> findContours(Mat grayImage) {
		List<MatOfPoint> mmContours = new ArrayList<MatOfPoint>();
		;
		Mat tempImage = new Mat();
		Mat mHierarchy = new Mat();	
		
		try {
			List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			grayImage.copyTo(tempImage);
			Imgproc.findContours(tempImage, contours, mHierarchy,
					Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

			// Find max contour area
			double maxArea = 0;
			Iterator<MatOfPoint> each = contours.iterator();
			while (each.hasNext()) {
				MatOfPoint wrapper = each.next();
				double area = Imgproc.contourArea(wrapper);
				if (area > maxArea)
					maxArea = area;
			}

			// Filter contours by area and resize to fit the original image size
			each = contours.iterator();
			while (each.hasNext()) {
				MatOfPoint contour = each.next();
				if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
					mmContours.add(contour);
				}
			}

		} catch (Exception e) {
		} finally {
			tempImage.release(); // free memory
		}
		return mmContours;
	}

	
	/**
	 * Calculates and returns homography matrix (with the help of a chessboard pattern).
	 * @param mRgba input image
	 * @return homography matrix
	 */
	public Mat getHomographyMatrix(Mat mRgba) {
		final Size mPatternSize = new Size(6, 9); // number of inner corners in
		// the used chessboard
		// pattern
		float x = -48.0f; // coordinates of first detected inner corner on
		// chessboard
		float y = 309.0f;
		float delta = 12.0f; // size of a single square edge in chessboard
		LinkedList<Point> PointList = new LinkedList<Point>();
		// Define real-world coordinates for given chessboard pattern:
		for (int i = 0; i < mPatternSize.height; i++) {
			y = 309.0f;
			for (int j = 0; j < mPatternSize.width; j++) {
				PointList.addLast(new Point(x, y));
				y += delta;
			}
			x += delta;
		}
		MatOfPoint2f RealWorldC = new MatOfPoint2f();
		RealWorldC.fromList(PointList);
		// Detect inner corners of chessboard pattern from image:
		Mat gray = new Mat();
		Imgproc.cvtColor(mRgba, gray, Imgproc.COLOR_RGBA2GRAY); // convert image
		// to grayscale
		MatOfPoint2f mCorners = new MatOfPoint2f();
		boolean mPatternWasFound = Calib3d.findChessboardCorners(gray,
				mPatternSize, mCorners);
		gray.release(); // free memory
		// Calculate homography:
		if (mPatternWasFound) {
			// Calib3d.drawChessboardCorners(mRgba, mPatternSize, mCorners,
			// mPatternWasFound); // for visualization
			return Calib3d.findHomography(mCorners, RealWorldC);
		} else
			return new Mat();
	}

	/**
	 * Filters the input image for the given color and opens the image (thus reducing noice).
	 * @param rgbaImage image to filter
	 * @param hsvColor color to filter for
	 * @return filtered image in grayscale
	 */
	public Mat filter(Mat rgbaImage, Scalar hsvColor) {
		Scalar mmLowerBound = new Scalar(0);
		Scalar mmUpperBound = new Scalar(0);
		Mat mmPyrDownMat = new Mat();
		Mat mmHsvMat = new Mat();
		Mat mmMask = new Mat();
		Mat mmDilatedMask = new Mat();

		try {

			Scalar mmColorRadius = new Scalar(30, 70, 70, 0); // Color radius
																// for range
																// checking in
																// HSV color
																// space
			double minH = (hsvColor.val[0] >= mmColorRadius.val[0]) ? hsvColor.val[0]
					- mmColorRadius.val[0]
					: 0;
			double maxH = (hsvColor.val[0] + mmColorRadius.val[0] <= 255) ? hsvColor.val[0]
					+ mmColorRadius.val[0]
					: 255;

			mmLowerBound.val[0] = minH;
			mmUpperBound.val[0] = maxH;

			mmLowerBound.val[1] = hsvColor.val[1] - mmColorRadius.val[1];
			mmUpperBound.val[1] = hsvColor.val[1] + mmColorRadius.val[1];

			mmLowerBound.val[2] = hsvColor.val[2] - mmColorRadius.val[2];
			mmUpperBound.val[2] = hsvColor.val[2] + mmColorRadius.val[2];

			mmLowerBound.val[3] = 0;
			mmUpperBound.val[3] = 255;

			Imgproc.pyrDown(rgbaImage, mmPyrDownMat);
			Imgproc.pyrDown(mmPyrDownMat, mmPyrDownMat);

			Imgproc.cvtColor(mmPyrDownMat, mmHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

			Core.inRange(mmHsvMat, mmLowerBound, mmUpperBound, mmMask);
//			Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
//					new Size(10, 10));
			Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
					new Size(15, 15));  // TODO: This should be a lot faster than working with a Circle; Test, if it's accurate enough.
										// TODO: Test the use of a bigger rectangle.
			Imgproc.dilate(mmMask, mmDilatedMask, element);
			Imgproc.erode(mmDilatedMask, mmDilatedMask, element);

			Imgproc.resize(mmDilatedMask, mmDilatedMask, rgbaImage.size());

		} catch (Exception e) {
		} finally {
			mmPyrDownMat.release();
			mmHsvMat.release();
			mmMask.release();
		}

		return mmDilatedMask;
	}
	
	/**
	 * computes center point of given contour
	 * 
	 * @param contours
	 * @return Point (representing center point)
	 */
	public Point computeCenterPt(List<MatOfPoint> contours) {
		if (contours.isEmpty()) {
			return (new Point(-200.0, -200.0)); // TODO: Better to return null
												// in this case?
		}
		double avgX = 0, avgY = 0;
		int count = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				avgX += p.x;
				avgY += p.y;
				count++;
			}
		}
		Point ptCenter = new Point(avgX / count, avgY / count);
		return ptCenter;
	}


	/**
	 * Calculates the distance between two points.
	 * @param p1 first point
	 * @param p2 second point
	 * @return distance in pixels
	 */
	public double distPointToPoint(Point p1, Point p2) {
		return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
	}

	/**
	 * measures distance from every point to center and computes average radius
	 * 
	 * @param contours list of contours
	 * @param center center of the given contours
	 * @return radius in pixels
	 */
	public double computeRadius(List<MatOfPoint> contours, Point center) {
		if (contours.isEmpty()) {
			return 0;
		}
		center = computeCenterPt(contours);
		double distToCenter = 0;
		int nrElements = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				distToCenter += distPointToPoint(p, center);
			}
			nrElements += pts.size();
		}
		distToCenter = distToCenter / nrElements;

		return distToCenter;
	}

	// TODO Needed?
	// TODO If so, comment
	private Mat drawBalls(Mat mRgbaWithBalls, List<Ball> foundBalls) {

		int counter = 0;
		for (Ball ball : foundBalls) {

			// Just for highlighting
			Scalar color = null;
			Scalar red = new Scalar(20);
			Scalar black = new Scalar(128);
			if (counter % 2 == 0) {
				color = red;
			} else {
				color = black;
			}

			Point center = ball.getBallCenterCameraFrame();
			int rad = (int) ball.getRadius();
			Core.circle(mRgbaWithBalls, center, 10, new Scalar(20), -1);
			Core.circle(mRgbaWithBalls, center, (int) rad, color, 5);
			Point lowestPoint = new Point(center.x, center.y + rad);
			Core.circle(mRgbaWithBalls, lowestPoint, 10, color, 5);
			// Imgproc.drawContours(img, ballArea, -1, color, -1);

			counter++;
		}

		return mRgbaWithBalls;
	}

	// TODO: circleCenters should not be updated globally; add a second method that does this every 15 frames (within MainActivity)
	/**
	 * Finds the centers of all circles on a given image matrix..
	 * @param mRgbaWork The image to find circles in.
	 * @param myColors a list of colors which should be processed
	 * 
	 * @return a list of centers of circles that are currently present on the
	 *         camera frame
	 */
	public List<Point> findCirclesOnCamera(Mat mRgbaWork, List<Scalar> myColors) {
		List<Point> circleCenters = new ArrayList<Point>();
		for (Scalar hsvColor : myColors) {
			Mat grayImg;
			do {
			grayImg = filter(mRgbaWork, hsvColor);
			}
			while (grayImg.empty());
			List<MatOfPoint> contours = findContours(grayImg);

			for (MatOfPoint area : contours) {

				List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
				ballArea.add(area);

				Point center = computeCenterPt(ballArea);

				circleCenters.add(center);
			}
			grayImg.release();
		}

		return circleCenters;
	}
	
	
	// TODO Add comment
	public List<Circle> findCirclesOnCamera2(Mat mRgbaWork, List<Scalar> myColors) {
		List<Circle> circlesList = new ArrayList<Circle>();
		for (Scalar hsvColor : myColors) {
			Mat grayImg;
			do {
				grayImg = filter(mRgbaWork, hsvColor);
			} while (grayImg.empty());
			List<MatOfPoint> contours = findContours(grayImg);

			for (MatOfPoint area : contours) {

				List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
				ballArea.add(area);

				Point center = computeCenterPt(ballArea);
				
				Double radius = computeRadius(ballArea, center);
				
				Circle foundCircle = new Circle(center, radius);

				circlesList.add(foundCircle);
			}
			grayImg.release();
		}
		return circlesList;
	}


	//TODO: method should work on circlesList
	// TODO Add comment
	public List<Square> findSquaresOnCamera(Mat mRgbaWork, List<Scalar> myColors) {
		List<Square> squareList = new ArrayList<Square>();
		int i = 0;
		for (Scalar hsvColor : myColors) {
			i++;
			Mat grayImg = new Mat();
			grayImg = filter(mRgbaWork, hsvColor);

			List<MatOfPoint> contours = findContours(grayImg);

			for (MatOfPoint area : contours) {
				List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();  // TODO rename
				ballArea.add(area);

				Point center = computeCenterPt(ballArea);

				Double halfHeight = squareHalfHeight(ballArea, center);
				
				Point lowerEdgeLeft = computeLowerEdgeLeft(ballArea, center); // TODO: Use result of squareHalfHeight (which should be renamed first)

				Square foundSquare = new Square(center, halfHeight, lowerEdgeLeft, i);

				squareList.add(foundSquare);
			}
		}
		Log.i(TAG, "(findSquaresOnCamera) Found squares: " + squareList.size());
		return squareList;
	}
	


	// TODO Add comment
	private Point computeLowerEdgeLeft(List<MatOfPoint> contours, Point center) {
		if (contours.isEmpty()) {
			return (new Point(-200.0, -200.0)); // TODO: Better to return null
												// in this case?
		}
		Double lowX = center.x;
		Double lowY = center.y;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				if(p.x < lowX)
					lowX = p.x;
				if(p.y < lowY)
					lowY = p.y;
			}
		}
		Log.i(TAG, "lowest edge left at x:"+lowX+" y:"+lowY);
		return new Point(lowX,lowY);
	}

	/**
	 * computes the difference of 2 points along the x-axis
	 * 
	 * @param one
	 *            point
	 * @param another
	 *            point
	 * @return absolute difference value
	 */
	public Double compare2PtbyX(Point a, Point b) {
		return Math.abs(a.x - b.x);
	}
	
	public Scalar getColorSalar(Mat mRgbaWork, Point pt){
		double[] color = mRgbaWork.get((int)pt.x,(int) pt.y);
		return new Scalar(color);
	}

	// TODO choose better name
	// TODO update Description
	/**
	 * compares alignment of all squares in global squareCenter-list and tries
	 * to find stacked squares if two squares are stacked the method deletes one
	 * of them and extends the first to the size of both
	 */
	private List<Beacon> findBeacon(List<Square> squareList, List<Beacon> beaconList) {
		beaconList = new ArrayList<Beacon>();
		Double TOL = 50.0;
		if (squareList.size() > 0) {
			for (int i=0;i<squareList.size()-1;i++) {
				for (int j=1;j<squareList.size();j++) {
					// it's not possible to write compare method in
					// point-class...
					if (compare2PtbyX(squareList.get(i).getCenter(),
							squareList.get(j).getCenter()) <= TOL) {
						Point newLowPt = new Point();
						Point newCenterPt = new Point(squareList.get(i).getCenter().x,
								(squareList.get(i).getCenter().y + squareList.get(j)
										.getCenter().y) / 2);
						Point newLowerLeftEdge = new Point();
						if (squareList.get(i).getCenter().y > squareList.get(j).getCenter().y) {
							newLowPt = squareList.get(j).getLowPt();
							newLowerLeftEdge = squareList.get(j).getLowerLeftEdge();
						} else {
							newLowPt = squareList.get(i).getLowPt();
							newLowerLeftEdge = squareList.get(i).getLowerLeftEdge();
						}
						// overwrite/extend one square to the size of both squares and
						// remove the second square form the list
						Integer upperColorID = null;
						beaconList.add(new Beacon(newCenterPt, newLowPt, newLowerLeftEdge, upperColorID, upperColorID));
					}
				}
			}
		}
		return beaconList;
	}
	
	
	// TODO: Needed? If so, add description
	// TODO: Rename and finalize (see TODOs within method)
	public Double squareHalfHeight(List<MatOfPoint> contours, Point center) {
		Double width = 0.0;
		Double height = 0.0;
		int countH = 0;
		int countW = 0;
		int count = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				width += Math.abs(p.x - center.x);
				height += Math.abs(p.y - center.y);
				count++;
			}
		}
		width = width / count;

		Double halfHeight = 0.0;
		Double halfWidth = 0.0;
		Double borderLeft = center.x - width;
		Double borderRight = center.x + width;
		Double borderTop = center.y + height;
		Double borderBottom = center.y - height;
		for (int j = 0; j < contours.size(); j++) {
			List<Point> pts = contours.get(j).toList();
			for (Point p : pts) {
				if (borderLeft <= p.x && p.x <= borderRight) {
					halfHeight += distPointToPoint(p, center);
					countH++;
				}
				if (borderBottom <= p.y && p.y <= borderTop) {
					halfWidth += distPointToPoint(p, center);
					countW++;
				}
			}
		}

		return halfHeight/countH;	// TODO also return halfWidth/countW
	}

}