package robot.opencv;

import java.util.ArrayList;
import java.util.Collections;
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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import robot.shapes.Ball;
import robot.shapes.Beacon;
import robot.shapes.BeaconSquareHolder;
import robot.shapes.Circle;
import robot.shapes.Square;
import android.util.Log;

public class ImageProcessor {

	// TODO Check if offsets for homography are needed

	// -> Beacon Calibration
	public static final Map<Integer, Point> BeaconPosition; // maps colorID
															// combo
	// (10*upperColorID +
	// lowerColorID) to the
	// position in real
	// world coordinates
	public static final Map<Integer, Integer> BeaconID;// maps colorID combo
	// (10*upperColorID +
	// lowerColorID) to the ID
	public static final Map<Integer, Integer> BeaconsAngleOffs; // maps
																// beaconID-combo
	// to angles which
	// have to be added
	// to the
	// calculation for
	// gaining the real
	// world coordinates

	static {
		Map<Integer, Point> tmpPosition = new LinkedHashMap<Integer, Point>();
		tmpPosition.put(12, new Point(-125.0, 125.0));
		tmpPosition.put(21, new Point(0.0, 125.0));
		tmpPosition.put(13, new Point(125.0, 125.0));
		tmpPosition.put(42, new Point(-125.0, 0.0));
		tmpPosition.put(24, new Point(125.0, 0.0));
		tmpPosition.put(41, new Point(-125.0, -125.0));
		tmpPosition.put(14, new Point(0.0, -125.0));
		tmpPosition.put(31, new Point(125.0, -125.0));
		BeaconPosition = Collections.unmodifiableMap(tmpPosition);

		Map<Integer, Integer> tmpID = new LinkedHashMap<Integer, Integer>();
		tmpID.put(12, 1);
		tmpID.put(21, 2);
		tmpID.put(13, 3);
		tmpID.put(42, 4);
		tmpID.put(24, 5);
		tmpID.put(41, 6);
		tmpID.put(14, 7);
		tmpID.put(31, 8);
		BeaconID = Collections.unmodifiableMap(tmpID);

		Map<Integer, Integer> tmpAngles = new LinkedHashMap<Integer, Integer>();
		tmpAngles.put(12, 90);
		tmpAngles.put(13, 90);
		tmpAngles.put(23, 90);
		tmpAngles.put(35, 180);
		tmpAngles.put(38, 180);
		tmpAngles.put(58, 180);
		tmpAngles.put(67, 270);
		tmpAngles.put(68, 270);
		tmpAngles.put(78, 270);
		tmpAngles.put(14, 0);
		tmpAngles.put(16, 0);
		tmpAngles.put(46, 0);
		tmpAngles.put(24, 45);
		tmpAngles.put(25, 135);
		tmpAngles.put(57, 225);
		tmpAngles.put(47, 315);
		BeaconsAngleOffs = Collections.unmodifiableMap(tmpAngles);
	}

	// <- Beacon Calibration

	private static double mMinContourArea = 0.1; // Minimum contour area in
													// percent for contours
													// filtering

	private String TAG; // Tag for log-messages sent to logcat

	/**
	 * Constructor method.
	 * 
	 * @param TAG
	 *            Used for messages to logcat.
	 */
	public ImageProcessor(String TAG) {
		this.TAG = TAG;
	}

	/**
	 * Finds contours of the objects within the given grayImage.
	 * 
	 * @param grayImage
	 *            input image
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
			int contourSize = contours.size();
			for (int i = 0; i < contourSize; i++) {
				MatOfPoint wrapper = contours.get(i);
				double area = Imgproc.contourArea(wrapper);
				if (area > maxArea)
					maxArea = area;
			}
			// while (each.hasNext()) {
			// MatOfPoint wrapper = each.next();
			// double area = Imgproc.contourArea(wrapper);
			// if (area > maxArea)
			// maxArea = area;
			// }

			// // Filter contours by area and resize to fit the original image
			// size
			// each = contours.iterator();
			// while (each.hasNext()) {
			// MatOfPoint contour = each.next();
			// if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
			// mmContours.add(contour);
			// }
			// }
			for (int j = 0; j < contourSize; j++) {
				MatOfPoint contour = contours.get(j);
				if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
					mmContours.add(contour);
				}
			}

		} catch (Exception e) {
		} finally {
			tempImage.release(); // free memory
			mHierarchy.release();
		}
		return mmContours;
	}

	/**
	 * Calculates and returns homography matrix (with the help of a chessboard
	 * pattern).
	 * 
	 * @param mRgba
	 *            input image
	 * @return homography matrix
	 */
	public Mat getHomographyMatrix(Mat mRgba) {
		final Size mPatternSize = new Size(6, 9); // number of inner corners in
		// the used chessboard
		// pattern
		float x = -95.0f; // coordinates of first detected inner corner on
		// chessboard
		float y = 224.0f;
		float delta = 22.0f; // size of a single square edge in chessboard
		LinkedList<Point> PointList = new LinkedList<Point>();
		// Define real-world coordinates for given chessboard pattern:
		double mPatternSizeHeight = mPatternSize.height;
		double mPatternSizeWidth = mPatternSize.width;
		for (int i = 0; i < mPatternSizeHeight; i++) {
			y = 224.0f;
			for (int j = 0; j < mPatternSizeWidth; j++) {
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
			Calib3d.drawChessboardCorners(mRgba, mPatternSize, mCorners,
					mPatternWasFound); // for visualization
			return Calib3d.findHomography(mCorners, RealWorldC);
		} else
			return new Mat();
	}

	/**
	 * Filters the input image for the given color and opens the image (thus
	 * reducing noice).
	 * 
	 * @param rgbaImage
	 *            image to filter
	 * @param hsvColor
	 *            color to filter for
	 * @param mode
	 *            'b' for filtering beacons (higher tolerances) or 'c' for
	 *            filtering circles
	 * @return filtered image in grayscale
	 */
	public Mat filter(Mat rgbaImage, Scalar hsvColor, char mode) {
		Scalar mmLowerBound = new Scalar(0);
		Scalar mmUpperBound = new Scalar(0);
		Mat mmPyrDownMat = new Mat();
		Mat mmHsvMat = new Mat();
		Mat mmMask = new Mat();
		Mat mmDilatedMask = new Mat();

		Scalar mmColorRadius = new Scalar(6, 40, 70, 0);

		switch (mode) {
		case 'b':
			mmColorRadius = new Scalar(10, 60, 255, 0); // Color radius
			// for range
			// checking in
			// HSV color
			// space
			break;
		case 'c':
			mmColorRadius = new Scalar(6, 40, 70, 0); // Color radius
			// for range
			// checking in
			// HSV color
			// space
			break;
		}

		try {

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
			// Mat element =
			// Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
			// new Size(10, 10));
			Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
					new Size(5, 5));
			Imgproc.dilate(mmMask, mmDilatedMask, element);
			Imgproc.erode(mmDilatedMask, mmDilatedMask, element);
			element.release();

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
	public Point computeCenterPt(MatOfPoint contours) {

		Moments mu = new Moments();
		mu = Imgproc.moments(contours, false);
		double x = mu.get_m10() / mu.get_m00();
		double y = mu.get_m01() / mu.get_m00();

		Point ptCenter = new Point(x, y);
		return ptCenter;
	}

	/**
	 * Calculates the distance between two points.
	 * 
	 * @param p1
	 *            first point
	 * @param p2
	 *            second point
	 * @return distance in pixels
	 */
	public double distPointToPoint(Point p1, Point p2) {
		return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
	}

	/**
	 * measures distance from every point to center and computes average radius
	 * 
	 * @param contours
	 *            list of contours
	 * @param center
	 *            center of the given contours
	 * @return radius in pixels
	 */
	public double computeRadius(MatOfPoint contours, Point center) {
		center = computeCenterPt(contours);
		double distToCenter = 0;
		List<Point> pts = contours.toList();
		int ptsAmount = pts.size();
		for (int i = 0; i < ptsAmount; i++) {
			distToCenter += distPointToPoint(pts.get(i), center);
		}
		distToCenter = distToCenter / pts.size();

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

	/**
	 * Finds the centers of all circles on a given image matrix..
	 * 
	 * @param mRgbaWork
	 *            The image to find circles in.
	 * @param myColors
	 *            a list of colors which should be processed
	 * 
	 * @return a list of centers of circles that are currently present on the
	 *         camera frame
	 */
	public List<Circle> findCirclesOnCamera2(Mat mRgbaWork,
			List<Scalar> myColors, List<Square> confirmedSquares) {
		List<Circle> circlesList = new ArrayList<Circle>();
		double colorAmount = myColors.size();
		for (int i = 0; i < colorAmount; i++) {
			Mat grayImg;
			grayImg = filter(mRgbaWork, myColors.get(i), 'c');
			List<MatOfPoint> contours = findContours(grayImg);
			grayImg.release();

			int contoursLength = contours.size();
			for (int j = 0; j < contoursLength; j++) {

				Point center = computeCenterPt(contours.get(j));

				Double radius = computeRadius(contours.get(j), center);

				Circle foundCircle = new Circle(center, radius);
				// check if circle is not in confirmedSquares-list
				if (checkCircleVsSquares(foundCircle, confirmedSquares))
					circlesList.add(foundCircle);
			}
		}
		Log.i(TAG,
				"found circles in findCirclesOnCamera2:" + circlesList.size());
		return circlesList;
	}

	/**
	 * checks if a circle is already detected as a square
	 * 
	 * @param foundCircle
	 * @param confirmedSquares
	 * @return true (true circle), false (circle is detected on a beacon)
	 */
	private boolean checkCircleVsSquares(Circle foundCircle,
			List<Square> confirmedSquares) {
		boolean unique = true;
		int squareAm = confirmedSquares.size();
		double TOL = 15.0;
		for (int i = 0; i < squareAm; i++) {
			if (distPointToPoint(foundCircle.getCenter(),
					confirmedSquares.get(i).center) < TOL)
				unique = false;
		}
		return unique;
	}

	/**
	 * detect contours for all colors in myColors-list and creates squares based
	 * on these contours
	 * 
	 * @param mRgbaWork
	 * @param myColors
	 * @return list of squares
	 */
	public List<Square> findSquaresOnCamera(Mat mRgbaWork, List<Scalar> myColors) {
		List<Square> squareList = new ArrayList<Square>();
		int colorAmount = myColors.size();
		for (int i = 0; i < colorAmount; i++) {
			Mat grayImg;
			grayImg = filter(mRgbaWork, myColors.get(i), 'b');

			List<MatOfPoint> contours = findContours(grayImg);

			grayImg.release();

			int contoursLength = contours.size();
			for (int j = 0; j < contoursLength; j++) {

				// Point center = computeCenterPt(contours.get(j));

				Square foundSquare = new Square(
						Imgproc.minAreaRect(new MatOfPoint2f(contours.get(j)
								.toArray())), i + 1);
				// double[] squareSize = squareSize(contours.get(j), center);

				// Point lowerEdgeLeft = computeLowerEdgeLeft(center,
				// squareSize);

				// Square foundSquare = new Square(center, squareSize[0],
				// lowerEdgeLeft, i + 1);

				/**
				 * squareSize[0] -> halfWidth squareSize[1] -> halfHeight
				 */
				squareList.add(foundSquare);

			}

		}
		// Log.i(TAG, "(findSquaresOnCamera) Found squares: " +
		// squareList.size());
		return squareList;
	}

	/**
	 * computes the coordinates for the lower left edge
	 * 
	 * @param center
	 *            point
	 * @param squareSize
	 *            (half width and half height)
	 * @return computed new lower left edge point
	 */
	private Point computeLowerEdgeLeft(Point center, double[] squareSize) {
		Double halfWidth = squareSize[0];
		Double halfHeight = squareSize[1];

		Log.i(TAG, "halfWidth:" + halfWidth + " halfHeight:" + halfHeight);
		Point lowestEdgeLeft = new Point(center.x - halfWidth, center.y
				+ halfHeight);

		return lowestEdgeLeft;
	}

	// TODO needed?
	// if so, add description
	public Scalar getColorSalar(Mat mRgbaWork, Point pt) {
		double[] color = mRgbaWork.get((int) pt.x, (int) pt.y);
		return new Scalar(color);
	}

	// TODO update Description
	/**
	 * compares alignment of all squares in global squareCenter-list and tries
	 * to find stacked squares if two squares are stacked the method deletes one
	 * of them and extends the first to the size of both
	 * 
	 * @param needs
	 *            list of all squares
	 */
	public BeaconSquareHolder findBeacons(List<Square> squareList) {
		List<Square> confSquares = new ArrayList<Square>(); // confSquares
															// contains
															// confirmed squares
		Collections.sort(squareList);
		// Log.i(TAG, "squareList: " + squareList.toString());
		List<Beacon> beaconList = new ArrayList<Beacon>();
		Double TOLx = 50.0;
		Double TOLy = 25.0;
		int squareListLength = squareList.size();
		for (int i = 0; i < squareListLength - 1; i++) {
			for (int j = i + 1; j < squareListLength; j++) {
				int squareFoundBelow = 0;
				Square squareA = squareList.get(i);
				Square squareB = squareList.get(j);
				Log.i(TAG,
						"Checking Squares against each other: "
								+ squareA.toString() + " and "
								+ squareB.toString());
				// squareA is always above squareB

				if (squareTest(squareA)) {
					Log.i(TAG, "upper Square passed the test");
				}

				if (twoSquaresMakeBeacon(squareA, squareB)) {
					Log.i(TAG, "the two squares make a beacon");
				}

				if (squareTest(squareA)
						&& twoSquaresMakeBeacon(squareA, squareB)) {

					Point newCenterPt = squareA.getLowPt();
					Size newSize;
					if (squareA.size.width > squareA.size.height) {
						newSize = new Size(squareA.size.width * 2,
								squareA.size.height);
					} else {
						newSize = new Size(squareA.size.width,
								squareA.size.height * 2);
					}
					// Point newLowerLeftEdge;
					// squareFoundBelow++;
					// newCenterPt = squareA.
					// newLowerLeftEdge = new
					// Point(squareA.getLowerLeftEdge().x,
					// squareA.getLowerLeftEdge().y
					// + (2 * squareA.getHalfHeight()));
					Beacon newBeacon = new Beacon(newCenterPt, newSize,
							squareA.angle, squareB.getColorID(),
							squareA.getColorID());
					if (checkIfNew(beaconList, newBeacon) &&
					// squareFoundBelow < 2
							// && checkIntersection(beaconList, newBeacon)
							BeaconID.get(newBeacon.getColorComb()) != null) {
						Log.i(TAG,
								"Made Beacon out of Square A: "
										+ squareA.toString()
										+ " and Square B: "
										+ squareB.toString());
						beaconList.add(newBeacon);
						confSquares.add(squareA);
						confSquares.add(squareB);
					}
				}
			}
		}
		return new BeaconSquareHolder(beaconList, confSquares);
	}

	// TODO: needed?
	/**
	 * checks if the detected beacon color id combination is correct
	 * 
	 * correct means the combination matches with one of the beacon color
	 * combinations we created
	 * 
	 * @param newBeacon
	 * @return true (newBeacon is a proper one) false otherwise
	 */
	private boolean checkBeaconColorComb(Beacon newBeacon) {
		int[] colorCombs = { 12, 21, 13, 42, 24, 41, 14, 31 };
		boolean correctId = false;
		int colorCombsSize = colorCombs.length;
		int refColor = newBeacon.getColorComb();
		for (int i = 0; i < colorCombsSize; i++) {
			if (colorCombs[i] == refColor)
				correctId = true;
		}
		return correctId;
	}

	private boolean twoSquaresMakeBeacon(Square squareA, Square squareB) {

		int TOL = 25;

		if (distPointToPoint(squareA.getLowPt(), squareB.getHighPt()) < TOL
				|| distPointToPoint(squareB.getLowPt(), squareA.getHighPt()) < TOL) {
			return true;
		}

		return false;
	}

	// TODO Needed? checkIntersection should be enough
	private boolean checkIfNew(List<Beacon> beaconList, Beacon newBeacon) {
		boolean unique = true;
		double TOL = 50;
		int beaconListLength = beaconList.size();
		for (int i = 0; i < beaconListLength; i++) {
			Point refPt = beaconList.get(i).center;
			if (distPointToPoint(refPt, newBeacon.center) < TOL) {
				unique = false;
			}
		}
		return unique;
	}

	// private boolean checkIntersection(List<Beacon> beaconList, Beacon
	// newBeacon) {
	// boolean noconflict = true;
	// int beaconListLength = beaconList.size();
	// for (int i = 0; i < beaconListLength; i++) {
	// double left = beaconList.get(i).getLowerLeftEdge().x;
	// double right = beaconList.get(i).getUpperRightEdge().x;
	// if (left <= newBeacon.getCenter().x
	// && newBeacon.getCenter().x <= right) {
	// noconflict = false;
	// }
	// }
	// return noconflict;
	// }

	/**
	 * tests if surrendered squares has the width and height relation to be a
	 * real square and not a circle
	 */
	private Boolean squareTest(Square s) {
		Double width = s.size.width;
		Double height = s.size.height;
		if (height > width) {
			if (height / width > 1.2) {
				return true;
			} else {
				return false;
			}
		} else {
			if (width / height > 1.2) {
				return true;
			} else {
				return false;
			}
		}

	}

	// TODO update description
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

	// TODO update description
	/**
	 * like compare2PtbyX, but only the y-axis
	 * 
	 * @param center
	 * @param center2
	 * @return
	 */
	private Double compare2PtbyY(Point a, Point b) {
		return Math.abs(a.y - b.y);
	}

	// TODO: Rename and finalize (see TODOs within method)
	public double[] squareSize(MatOfPoint contours, Point center) {
		Double width = 0.0, height = 0.0;
		int countH = 0, countW = 0, count = 0;
		List<Point> pts = contours.toList();
		int ptsAmount = pts.size();
		for (int i = 0; i < ptsAmount; i++) {
			width += Math.abs(pts.get(i).x - center.x);
			height += Math.abs(pts.get(i).y - center.y);
			count++;
		}

		width = width / count;
		height = height / count;

		Double halfHeight = 0.0, halfWidth = 0.0;
		Double borderLeft = center.x - width;
		Double borderRight = center.x + width;
		Double borderTop = center.y + height;
		Double borderBottom = center.y - height;

		for (int i = 0; i < ptsAmount; i++) {
			if (borderLeft <= pts.get(i).x && pts.get(i).x <= borderRight) {
				halfHeight += Math.abs(pts.get(i).y - center.y);
				countH++;
			}
			if (borderBottom <= pts.get(i).y && pts.get(i).y <= borderTop) {
				halfWidth += Math.abs(pts.get(i).x - center.x);
				countW++;
			}
		}
		double[] squareSize = new double[2];
		squareSize[0] = halfWidth / countW;
		squareSize[1] = halfHeight / countH;
		return squareSize;
	}

}