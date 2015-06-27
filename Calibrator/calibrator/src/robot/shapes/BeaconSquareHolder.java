package robot.shapes;

import java.util.ArrayList;
import java.util.List;

public class BeaconSquareHolder {

	private List<Beacon> beaconList = new ArrayList<Beacon>();
	private List<Square> squareList = new ArrayList<Square>();

	public BeaconSquareHolder(List<Beacon> beaconList, List<Square> squareList) {
		this.beaconList = beaconList;
		this.squareList = squareList;
	}

	public List<Beacon> getBeaconList() {
		return beaconList;
	}

	public List<Square> getSquareList() {
		return squareList;
	}

}
