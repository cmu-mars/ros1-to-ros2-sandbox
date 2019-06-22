package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.List;
import ros2sy.logic.Encoding;
import ros2sy.logic.EncodingUtil;
import ros2sy.logic.SequentialEncoding;
import uniol.apt.adt.pn.PetriNet;
import ros2sy.logic.Variable;

public class Synthesis {

	public static ArrayList<ArrayList<String>> synthesizeAll(PetriNet net, List<String> inputs, int max_loc) {
		int loc = 1;
		
		ArrayList<ArrayList<String>> stringss = new ArrayList<ArrayList<String>>();

		while (loc <= max_loc) {
			// create a formula that has the same semantics as the petri-net
			Encoding encoding = new SequentialEncoding(net, loc);

			// set initial state and final state
			encoding.setState(EncodingUtil.setInitialState(net, inputs), 0);

			// reachability analysis
			List<Variable> result = Encoding.solver.findPath(loc);
			while (!result.isEmpty()) {
				System.out.println("============================");
				stringss.add(new ArrayList<String>());
				int index = stringss.size() - 1;
				for (Variable s : result) {
					stringss.get(index).add(s.getName());
					System.out.println(s.getName());
				}
				result = Encoding.solver.findPath(loc);
			}
			loc++;
		}
		
		return stringss;
	}

	public static void main(String[] args) {

	}

}
