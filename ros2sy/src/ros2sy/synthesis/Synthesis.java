package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import ros2sy.code.CppCode;
import ros2sy.code.InputVariables;
import ros2sy.code.SketchFiller;
import ros2sy.json.ParseJson;
import ros2sy.logic.Encoding;
import ros2sy.logic.EncodingUtil;
import ros2sy.logic.SequentialEncoding;
import uniol.apt.adt.pn.PetriNet;
import ros2sy.logic.Variable;
import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.Method;
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
	
	public static void main(String[] args) throws Exception {
		String [] searchSpaces = {"node", "publisher", "rclcpp", "subscription"};
		
		ArrayList<Method> methods = new ArrayList<Method>();
		for (int i = 0; i < searchSpaces.length; i++) {
			String fileName = "scrape_rclcpp_docs/jsons/" + searchSpaces[i] + ".json";
			methods.addAll(ParseJson.parseOutMethods(fileName));
			System.out.println("Methods length: " + Integer.toString(methods.size()));
		}
		
		ArrayList<String> dontClone = new ArrayList<>();
		dontClone.add("void");
		
		MethodsToPetriNet mtpn = new MethodsToPetriNet(methods, dontClone);
		
		MethodsToPetriNet.createDotFile(mtpn.getNet(), "dot/full_search.dot");
		
		
		SearchSpace search = new SearchSpace(methods, "scrape_rclcpp_docs/tags.json", mtpn);
		
		
		search.addBlock("initialization");
		search.addBlock("node", "constructor");
		search.addToLastBlock("subscription", "constructor");
		search.addBlock("spin");
		
		
		ArrayList<String> neverUse = search.getNameIntersect("shutdown");
		
		
		InputVariables ivs = new InputVariables();
		
		HashMap<String, String> varsToTypes = ParseJson.getInputVariableToType("inputs/listener_input.json");
		
		for (Map.Entry<String, String> e : varsToTypes.entrySet()) {
			ivs.addInput(e.getKey(), e.getValue());
		}
		
		ArrayList<ArrayList<String>> inputs = ParseJson.getInputTypesFromFile("inputs/listener_input.json");
		
		ArrayList<ArrayList<String>> correctAnswers = ParseJson.getCorrectAnswersFromFile("inputs/correct-answers.json");

		
		ArrayList<String> correctSequence = new ArrayList<String>();
		
		for (int i = 0; i < search.numBlocks(); i++) {
			List<List<String>> k = search.getBlock(i);

			ArrayList<String> dontUse = search.othersExcept(i, neverUse);
						
			System.out.print("Block: ");
			System.out.println(k);
			System.out.println(dontUse);
			
			ArrayList<ArrayList<String>> strss = Synthesis.synthesizeAll(mtpn.getNet(), inputs.get(i), 3, k, dontUse);
			System.out.print("Number of possibilities generated for block ");
			
			
			String blocksSubstring = k.toString().substring(0, Math.min(k.toString().length(), 54));
			blocksSubstring = blocksSubstring + ((blocksSubstring.length() < k.toString().length()) ? "...]" : "");
			System.out.print(blocksSubstring);
			System.out.print(": ");
			System.out.println(strss.size());
			if (strss.size() > 0) {
				correctSequence.addAll(strss.get(0));
			}
			
			int maxShown = 10;
			
			System.out.println("The top " + Integer.toString(maxShown) + ":");
			for (ArrayList<String> strs : strss.subList(0, Math.min(strss.size(), maxShown))) {
				System.out.println(strs);
			}
			
			for (ArrayList<String> strs : strss) {
				boolean hasEverything = true;
				for (String s : correctAnswers.get(i)) {
					hasEverything = hasEverything && strs.contains(s);
				}
				if (hasEverything) {
					int indexOfStrs = strss.indexOf(strs);
					System.out.println("Found the correct answer at " + Integer.toString(indexOfStrs) + "/" + Integer.toString(strss.size()) + ":");
					System.out.println(strs);
					break;
				}
			}
		}
		
		SketchFiller filler = new SketchFiller(mtpn, correctSequence);
		
		filler.fillSketches("ex1/sketches/listener.sketch", ivs);
	}
}
