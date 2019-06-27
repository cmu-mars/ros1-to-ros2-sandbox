package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import ros2sy.logic.Encoding;
import ros2sy.logic.EncodingUtil;
import ros2sy.logic.SequentialEncoding;
import uniol.apt.adt.pn.PetriNet;
import ros2sy.logic.Variable;
import ros2sy.synthesis.*;

public class Synthesis {
	
	public static ArrayList<ArrayList<String>> synthesizeAll(PetriNet net, List<String> inputs, int max_loc) {
		List<List<String>> k = new ArrayList<>();
		ArrayList<String> k1 = new ArrayList<>();
		k1.add("rclcpp::init");
		k.add(k1);
		
		return Synthesis.synthesizeAll(net, inputs, max_loc, k, new ArrayList<String>());
	}

	public static ArrayList<ArrayList<String>> synthesizeAll(PetriNet net, List<String> inputs, int max_loc, List<List<String>> k, List<String> prevent) {
		int loc = 1;

		List<Result> unsorted_result = new ArrayList<>();
		ArrayList<ArrayList<String>> sorted_result = new ArrayList<ArrayList<String>>();

		while (loc <= max_loc) {
			// create a formula that has the same semantics as the petri-net
			Encoding encoding = new SequentialEncoding(net, loc);

			// set initial state and final state
			encoding.setState(EncodingUtil.setInitialState(net, inputs), 0);

			// example on how to add some information from the previous code
			
			encoding.refactorInfo(k);
			encoding.doesNotOccur(prevent);
			
			
			// reachability analysis
			List<Variable> result = Encoding.solver.findPath(loc);
			while (!result.isEmpty()) {
//				 System.out.println("============================");
				ArrayList<String> api_result = new ArrayList<>();
				for (Variable s : result) {
					api_result.add(s.getName());
//					 System.out.println(s.getName());
				}
//				System.out.println("Cost = " + Encoding.solver.getCost());
				Result r = new Result(api_result, Encoding.solver.getCost());
				unsorted_result.add(r);
				result = Encoding.solver.findPath(loc);
			}
			loc++;
		}

		Collections.sort(unsorted_result, new ResultComp());

		// Either use the sorted version or the copy to the new array
		for (Result res : unsorted_result) {
			ArrayList<String> ss = new ArrayList<>();
			ss.addAll(res.apis);
			sorted_result.add(ss);
		}

		for (List<String> l : sorted_result) {
//			System.out.println("============================");
			for (String s : l) {
//				System.out.println(s);
			}
		}

		return sorted_result;
	}
}
