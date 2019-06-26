package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.List;

public class Result {

	public List<String> apis = new ArrayList<>();
	public int cost;

	public Result(List<String> apis, int cost) {
		this.apis.addAll(apis);
		this.cost = cost;
	}
	
	@Override
    public String toString() {
    	return apis.toString() + "|w=" + cost;
    }

}
