package robot.navigate;

public class Position {
	int x;
	int y;
	int theta;
	
	public Position(int x,int y,int theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	
	public int getX() {
		return x;
	}
	
	public void setX(int x) {
		this.x = x;
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
	}
	public int getTheta() {
		return theta;
	}
	public void setTheta(int theta) {
		this.theta = theta;
	}
}
