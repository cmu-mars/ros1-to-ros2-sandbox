package ros2sy.synthesis;

import java.util.Comparator;

public class ResultComp implements Comparator<Result> {

	@Override
	public int compare(Result e1, Result e2) {
		if (e1.cost < e2.cost)
			return -1;
		else if (e1.cost == e2.cost && e1.apis.size() < e2.apis.size())
			return -1;
		else if (e1.cost == e2.cost && e1.apis.size() == e2.apis.size()) {
			return 0;
		}
		else
			return 1;
	}
}