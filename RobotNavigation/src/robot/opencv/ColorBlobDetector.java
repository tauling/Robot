package robot.opencv;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.util.Log;

// TODO: Review this class: Remove all unnecessary methods
// TODO: Review this class: Make methods private/public

public class ColorBlobDetector {
	// Lower and Upper bounds for range checking in HSV color space
	private List<Scalar> mLowerBound = new ArrayList<Scalar>();
	private List<Scalar> mUpperBound = new ArrayList<Scalar>();
	// Minimum contour area in percent for contours filtering
	private static double mMinContourArea = 0.1;
	// Color radius for range checking in HSV color space
	private Scalar mColorRadius = new Scalar(25, 60, 60, 0);
	private List<Mat> mSpectrum = new ArrayList<Mat>();
	private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

	// Cache
	Mat mPyrDownMat = new Mat();
	Mat mHsvMat = new Mat();
	List<Mat> mMask= new ArrayList<Mat>();
	List<Mat> mDilatedMask = new ArrayList<Mat>();
	Mat mHierarchy = new Mat();
	
	
	private static final String TAG = "OCVSample::Activity";
	
	public void setColorRadius(Scalar radius) {
		mColorRadius = radius;
	}

	public void setHsvColor(List<Scalar> hsvColor) {
		int oldSize = mLowerBound.size();
		//init Lower and UpperBound
		for(int i=hsvColor.size()-mLowerBound.size();i>0;i--){
			mLowerBound.add(new Scalar(0));
			mUpperBound.add(new Scalar(0));
			mSpectrum.add(new Mat());
			mMask.add(new Mat());
			mDilatedMask.add(new Mat());
		}
		Log.i(TAG, "colors saved in detector: "+mUpperBound.size());
		//fill them with values
		for(int j=oldSize;j<hsvColor.size();j++){
			double minH = (hsvColor.get(j).val[0] >= mColorRadius.val[0]) ? hsvColor.get(j).val[0]
					- mColorRadius.val[0]
					: 0;
			double maxH = (hsvColor.get(j).val[0] + mColorRadius.val[0] <= 255) ? hsvColor.get(j).val[0]
					+ mColorRadius.val[0]
					: 255;
	
			mLowerBound.get(j).val[0] = minH;
			mUpperBound.get(j).val[0] = maxH;
	
			mLowerBound.get(j).val[1] = hsvColor.get(j).val[1] - mColorRadius.val[1];
			mUpperBound.get(j).val[1] = hsvColor.get(j).val[1] + mColorRadius.val[1];
	
			mLowerBound.get(j).val[2] = hsvColor.get(j).val[2] - mColorRadius.val[2];
			mUpperBound.get(j).val[2] = hsvColor.get(j).val[2] + mColorRadius.val[2];
	
			mLowerBound.get(j).val[3] = 0;
			mUpperBound.get(j).val[3] = 255;


		Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

		for (int h= 0; h < maxH - minH; h++) {
			byte[] tmp = { (byte) (minH + h), (byte) 255, (byte) 255 };
			spectrumHsv.put(0, h, tmp);
		}

		Imgproc.cvtColor(spectrumHsv, mSpectrum.get(j), Imgproc.COLOR_HSV2RGB_FULL, 4);
		}
	}

	public List<Mat> getSpectrum() {
		return mSpectrum;
	}

	public void setMinContourArea(double area) {
		mMinContourArea = area;
	}

	public void process(Mat rgbaImage) {
		Imgproc.pyrDown(rgbaImage, mPyrDownMat);
		Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

		Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		for(int i=0;i<mLowerBound.size();i++){
			Core.inRange(mHsvMat, mLowerBound.get(i), mUpperBound.get(i), mMask.get(i));
			Log.i(TAG, i+". color"+mLowerBound.get(i)+mUpperBound.get(i));
			Imgproc.dilate(mMask.get(i), mDilatedMask.get(i), new Mat());

		Imgproc.findContours(mDilatedMask.get(i), contours, mHierarchy,
				Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		}
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
		mContours.clear();
		each = contours.iterator();
		while (each.hasNext()) {
			MatOfPoint contour = each.next();
			if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
				Core.multiply(contour, new Scalar(4, 4), contour);
				mContours.add(contour);
			}
		}
	}

	public List<MatOfPoint> getContours() {
		return mContours;
	}

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

		// Calculate homography:
		if (mPatternWasFound) {
//			Calib3d.drawChessboardCorners(mRgba, mPatternSize, mCorners,
//					mPatternWasFound); // for visualization
			return Calib3d.findHomography(mCorners, RealWorldC);
		} else
			return new Mat();
	}
}
