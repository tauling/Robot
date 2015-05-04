package robot.opencv;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
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

// TODO: Review this class: Make methods private/public

public class ColorBlobDetector {
	// Minimum contour area in percent for contours filtering
	private static double mMinContourArea = 0.1;

	// Cache
	Mat mPyrDownMat = new Mat();
	Mat mHsvMat = new Mat();
	Mat mMask = new Mat();
	Mat mDilatedMask = new Mat();
	Mat mHierarchy = new Mat();

	// TODO add comment
	public List<MatOfPoint> findContours(Mat grayImage) {
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		String TAG = "OCVSample::Activity";
		Log.i(TAG, "Typ " + grayImage.type());
		Mat tempImage = new Mat();
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
		List<MatOfPoint> mmContours = new ArrayList<MatOfPoint>();
		each = contours.iterator();
		while (each.hasNext()) {
			MatOfPoint contour = each.next();
			if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
				mmContours.add(contour);
			}
		}
		//try to free memory
		tempImage.release();
		return mmContours;
	}

	// TODO add comment
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
		////try to free memory
		gray.release();
		// Calculate homography:
		if (mPatternWasFound) {
			// Calib3d.drawChessboardCorners(mRgba, mPatternSize, mCorners,
			// mPatternWasFound); // for visualization
			return Calib3d.findHomography(mCorners, RealWorldC);
		} else
			return new Mat();
	}

	// TODO add description
	public Mat filter(Mat rgbaImage, Scalar hsvColor) {
		Scalar mmLowerBound = new Scalar(0);
		Scalar mmUpperBound = new Scalar(0);
		Mat mmPyrDownMat = new Mat();
		Mat mmHsvMat = new Mat();
		Mat mmMask = new Mat();
		Mat mmDilatedMask = new Mat();
		// Color radius for range checking in HSV color space
		Scalar mmColorRadius = new Scalar(30, 70, 70, 0);
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
		Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
				new Size(10, 10));

		Imgproc.dilate(mmMask, mmDilatedMask, element);
		Imgproc.erode(mmDilatedMask, mmDilatedMask, element);

		Imgproc.resize(mmDilatedMask, mmDilatedMask, rgbaImage.size());
		//tries to 3 Mat objects
		mmPyrDownMat.release();
		mmHsvMat.release();
		mmMask.release();
		
		return mmDilatedMask;
	}

}