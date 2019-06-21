package ros2sy.synthesis;

import java.util.List;
import ros2sy.logic.Encoding;
import ros2sy.logic.EncodingUtil;
import ros2sy.logic.SequentialEncoding;
import uniol.apt.adt.pn.PetriNet;
import ros2sy.logic.Variable;

public class Synthesis {

	public static void synthesizeAll(PetriNet net, List<String> inputs, int max_loc) {
		int loc = 1;

		while (loc <= max_loc) {
			// create a formula that has the same semantics as the petri-net
			Encoding encoding = new SequentialEncoding(net, loc);

			// set initial state and final state
			encoding.setState(EncodingUtil.setInitialState(net, inputs), 0);

			// reachability analysis
			List<Variable> result = Encoding.solver.findPath(loc);
			while (!result.isEmpty()) {
				System.out.println("============================");
				for (Variable s : result) {
					System.out.println(s.getName());
				}
				result = Encoding.solver.findPath(loc);
			}
			loc++;
		}
	}

	public static void main(String[] args) {

	}

}
