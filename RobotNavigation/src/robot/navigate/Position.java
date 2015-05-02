package robot.navigate;

public class Position implements Comparable<Object> {
	public double x;
	public double y;
	public int theta;

	public Position(double x, double y, int theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
	}

	public String toString() {
		return "{" + this.x + "|" + this.y + "|" + this.theta + "}";
	}

	public double minus(Position two) {
		return Math.sqrt(Math.pow(this.x - two.x, 2)
				+ Math.pow(this.y - two.y, 2));
	}

	@Override
	public int compareTo(Object another) {
		if (this.x == ((Position) another).x
				&& this.y == ((Position) another).y)
			return 0;
		else if (this.x != ((Position) another).x
				|| this.y != ((Position) another).y)
			return 1;
		else
			return -1;
	}
}
