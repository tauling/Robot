package robot.shapes;

import org.opencv.core.Point;

public class Ball {
	private Integer id;
	private Point posGroundPlane;
	private Point ballCenterCameraFrame;
	private double radius;
	public static int counter = 0;

	public Ball(Point ballCenterCameraFrame, Point posGroundPlane, double radius) {
		this.id = counter++;
		this.posGroundPlane = posGroundPlane;
		this.ballCenterCameraFrame = ballCenterCameraFrame;
		this.radius = radius;
	}

	public Integer getId() {
		return id;
	}

	public Point getPosGroundPlane() {
		return posGroundPlane;
	}

	public void setPosGroundPlane(Point posGroundPlane) {
		this.posGroundPlane = posGroundPlane;
	}

	public Point getBallCenterCameraFrame() {
		return ballCenterCameraFrame;
	}

	public void setBallCenterCameraFrame(Point ballCenterCameraFrame) {
		this.ballCenterCameraFrame = ballCenterCameraFrame;
	}

	public double getRadius() {
		return radius;
	}

	public void setRadius(int radius) {
		this.radius = radius;
	}

	public String toString() {
		return "id: " + this.id + "\nposition ground plane"
				+ this.posGroundPlane + "\nballCenterCameraFrame: "
				+ this.ballCenterCameraFrame + "\nradius: " + this.radius;
	}

}
