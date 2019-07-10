package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

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
	private static final Logger LOGGER = LogManager.getLogger(Synthesis.class.getName());
	
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
				LOGGER.trace("============================");
				ArrayList<String> api_result = new ArrayList<>();
				for (Variable s : result) {
					api_result.add(s.getName());
				  	LOGGER.trace(s.getName());
				}
			  LOGGER.trace("Cost = {}", Encoding.solver.getCost());
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
			LOGGER.trace("============================");
			for (String s : l) {
				LOGGER.trace(s);
				if (prevent.contains(s)) {
					LOGGER.error("Result contains unwanted transition: {}", s);
				}
			}
		}

		return sorted_result;
	}
	
	public static void checkAllSolutions(int i, ArrayList<ArrayList<String>> strss, ArrayList<ArrayList<String>> correctAnswers) {
		for (ArrayList<String> strs : strss) {
			boolean hasEverything = true;
			for (String s : correctAnswers.get(i)) {
				hasEverything = hasEverything && strs.contains(s);
			}
			if (hasEverything) {
				int indexOfStrs = strss.indexOf(strs);
				LOGGER.info("Found the correct answer at {}/{}: {}", Integer.toString(indexOfStrs), Integer.toString(strss.size()), strs);
				break;
			}
		}
	}
	
	public static void main(String[] args) throws Exception {
		System.setProperty("log4j.configurationFile", "ros2sy/src/resources/log4j2.xml");

		ArrayList<Method> methods = ParseJson.getAllMethods("node", "publisher", "rclcpp", "subscription");
		
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
						
			LOGGER.debug("Block: {}", k);
//			LOGGER.trace(dontUse);
			
			ArrayList<ArrayList<String>> strss = Synthesis.synthesizeAll(mtpn.getNet(), inputs.get(i), 3, k, dontUse);
			
			
			String blocksSubstring = k.toString().substring(0, Math.min(k.toString().length(), 54));
			blocksSubstring = blocksSubstring + ((blocksSubstring.length() < k.toString().length()) ? "...]" : "") + ": ";
			LOGGER.debug("Number of possibilities generated for block {}{}", blocksSubstring, strss.size());
			
			if (strss.size() > 0) {
				correctSequence.addAll(strss.get(0));
			}
			
			int maxShown = Math.min(strss.size(), 10);
			
			LOGGER.debug("The top {}:", maxShown);
			for (ArrayList<String> strs : strss.subList(0, maxShown)) {
				LOGGER.debug(strs);
			}
			
			Synthesis.checkAllSolutions(i, strss, correctAnswers);
		}
		
		SketchFiller filler = new SketchFiller(mtpn, correctSequence);
		
		filler.fillSketches("ex1/sketches/listener.sketch", ivs);
	}
}
