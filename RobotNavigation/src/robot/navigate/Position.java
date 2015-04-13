package robot.navigate;

public class Position implements Comparable{
	double x;
	double y;
	int theta;
	
	public Position(double x,double y,int theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	
	public double getX() {
		return x;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	public double getY() {
		return y;
	}
	public void setY(double y) {
		this.y = y;
	}
	public int getTheta() {
		return theta;
	}
	public void setTheta(int theta) {
		this.theta = theta;
	}

	public double minus(Position two){
		return Math.sqrt(Math.pow(this.x - two.getX(), 2) + Math.pow(this.y - two.getY(), 2));
	}
	
	@Override
	public int compareTo(Object another) {
		if (this.x == ((Position) another).x && this.y == ((Position) another).y)
            return 0;
        else if (this.x != ((Position) another).x || this.y != ((Position) another).y)
            return 1;
        else
            return -1;
	}
}
