package ros2sy.json;

import java.util.*;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;

import com.google.gson.JsonParser;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;

import ros2sy.code.CppCode;
import ros2sy.petri.*;
import ros2sy.sig.*;
import ros2sy.synthesis.Synthesis;
import uniol.apt.adt.pn.*;

/**
 * Class that contains methods to parse out Method objects
 * from a JSON file.
 * 
 * Also contains a main method that takes our example search
 * space, creates a Petri Net from it, and then creates a dot
 * file that displays the petri net.
 * 
 * @author audrey
 *
 */
public class ParseJson {
	
	public ParseJson(String fileName) {
		
	}
	
	/**
	 * Gets an ArrayList of String objects from the given JsonObject,
	 * provided that this JsonObject has the "args" key available,
	 * and the value associated with "args" is a JsonArray.
	 * 
	 * @param jo		a JsonObject
	 * @return			an ArrayList containing the contents of the
	 * 							"args" keyword, if it existed in the JsonObject
	 */
	private static ArrayList<String> getArgsList(JsonObject jo) {
		ArrayList<String> args = new ArrayList<String>();
		if (jo.has("args")) {			
			JsonArray mArgs = jo.get("args").getAsJsonArray();
			Iterator<JsonElement> iter = mArgs.iterator();
			while (iter.hasNext()) {
				args.add(iter.next().getAsString());
//				System.out.println(args.get(args.size() - 1));
			}
		}
		
		return args;
	}
	
	/**
	 * Recursively descends into a JsonElement and parses out a
	 * list of methods, building the list of methods recursively.
	 * 
	 * @param key		the String key that is associated with the JsonElement
	 * @param elmt	a JsonElement to descend into
	 * @return			an ArrayList of Method objects, parsed from the JsonElement
	 */
	private static ArrayList<Method> unwrap(String key, JsonElement elmt) {
		ArrayList<Method> methods = new ArrayList<Method>();
//		System.out.println("key: " + key);
//		System.out.println(elmt.isJsonObject());
		
		if (elmt.isJsonObject()) {
			JsonObject jo = elmt.getAsJsonObject();
//			System.out.println(jo.has("args"));
  			if (jo.has("ros2")) {
  				
  				ArrayList<Method> unwrappedMethods = ParseJson.unwrap("ros2", jo.get("ros2"));
  				
  				
  				if (jo.has("ros1")) {
  					if (jo.get("ros1").isJsonObject()) {
  						JsonObject ros1 = jo.get("ros1").getAsJsonObject();
  						if (ros1.keySet().size() == 1) {
  							for (String ros1Key : ros1.keySet()) {
  								for (Method m : unwrappedMethods) {
  									m.setRos1Name(ros1Key);
  								}
  							}
  						}
  					}
  				}
  				
  				methods.addAll(unwrappedMethods);
			} else if (jo.has("args")) {
				ArrayList<String> args = ParseJson.getArgsList(jo);
				
				if (jo.has("func_type")) {
					String funcType = jo.get("func_type").getAsString();
					if (jo.has("return")) {
						methods.add(new Method(key, args, jo.get("return").getAsString(), funcType));
					} else {
						methods.add(new Method(key, args, "", funcType));						
					}
				} else {
					if (jo.has("return")) {
						String returnType = jo.get("return").getAsString();
						methods.add(new Method(key, args, returnType));
					}
				}
			} else {
				Set<String> keys = jo.keySet();
				
				for (String subKey : keys) {
					// this brings us to the code for each line
					methods.addAll(ParseJson.unwrap(subKey, jo.get(subKey)));
				}
			}
			
		} else if (elmt.isJsonArray()) {
			JsonArray jarray = elmt.getAsJsonArray();
			Iterator<JsonElement> iter = jarray.iterator();
			
			while (iter.hasNext()) {
				methods.addAll(ParseJson.unwrap(key, iter.next()));
			}
		}
		
		return methods;
	}
	
	/**
	 * Parse out Method objects from a JSON file.
	 * 
	 * @param fileName		the name of the file to parse
	 * @return						an ArrayList containing all of the methods parsed out
	 * 									from the JSON file
	 */
	public static ArrayList<Method> parseOutMethods(String fileName) {
		ArrayList<Method> methods = new ArrayList<Method>();
		try {
			FileReader fr = new FileReader(fileName);
			
			JsonParser jp = new JsonParser();
			
			try {
				JsonElement je = jp.parse(fr);
				
				if (je.isJsonObject()) {
					JsonObject jobj = je.getAsJsonObject();
					Set<String> keys = jobj.keySet();
					
					
					for (String key : keys) {
						// this brings us to the code for each line
						methods.addAll(ParseJson.unwrap(key,  jobj.get(key)));
					}
				}
			} catch (JsonParseException e) {
				System.out.println("Json parse had an exception.");
				System.out.println(e);
			}
		} catch (FileNotFoundException e) {
			System.out.println("Caught a FileNotFoundException in parseOutMethods:");
			System.out.println(e);
			
			System.out.println("Could not extract methods from JSON. Could not find json file " + fileName);
		}
		
		
		return methods;
	}
	
	/**
	 * Take the example search space json file that I created, and
	 * then parse out the json, extracts the methods described as
	 * Method objects, and create a PetriNet from this.
	 * 
	 * The PetriNet is then output as a DOT file.
	 * 
	 * @param args				arguments from the command line
	 * @throws Exception	
	 */
	public static void main(String[] args) throws Exception {
		ArrayList<Method> methods = ParseJson.parseOutMethods("ex1/listener-search-space.json");
		
//		System.out.println("Methods: ");
//		System.out.println(Integer.toString(methods.size()));
//		for (Method m : methods) {
//			System.out.println(m);
//		}
		
		ArrayList<String> sillyToClone = new ArrayList<String>();
		sillyToClone.add("void");
		
		// We're just going block these up in a silly way.
		
		MethodsToPetriNet mtpn = new MethodsToPetriNet(methods, sillyToClone);
		HashMap<String, ArrayList<String>> blockByRos1Name = new HashMap<String, ArrayList<String>>();
		PetriNet pn = mtpn.getNet();
		
		for (Method m : methods) {
			if (!blockByRos1Name.containsKey(m.ros1Name)) {
				blockByRos1Name.put(m.ros1Name, new ArrayList<String>());
			}
			blockByRos1Name.get(m.ros1Name).addAll(mtpn.getNicknamesOfMethod(m));
		}
		
		ArrayList<ArrayList<String>> blocks = new ArrayList<ArrayList<String>>();
		
		// Our three blocks. This will be figured out programmatically in the
		// future.
		blocks.add(new ArrayList<String>());
		blocks.get(0).add("ros::init");
		blocks.add(new ArrayList<String>());
		blocks.get(1).add("ros::NodeHandle");
		blocks.get(1).add("ros::NodeHandle::subscribe");
		blocks.add(new ArrayList<String>());
		blocks.get(2).add("ros::spin");
//		blocks.add(blockByRos1Name.get("ros::init"));
//		blocks.add(new ArrayList<String>(blockByRos1Name.get("ros::NodeHandle")));
//		blocks.get(1).addAll(blockByRos1Name.get("ros::NodeHandle::subscribe"));
//		blocks.add(blockByRos1Name.get("ros::spin"));
		
		// Dummy input array for testing the synthesis aspect
		ArrayList<String> input = new ArrayList<String>();
		input.add("char const *const");
		input.add("int");
		
		ArrayList<String> neverUse = new ArrayList<String>();
		neverUse.add("rclcpp::shutdown");
		
		ArrayList<ArrayList<String>> inputs = new ArrayList<>();
		inputs.add(input);
		inputs.add(new ArrayList<String>());
		inputs.get(1).add("std::string");
		inputs.get(1).add("const std::string&");
		inputs.get(1).add("const rclcpp::QoS&");
		inputs.get(1).add("CallbackT &&");
		inputs.add(new ArrayList<String>(inputs.get(1)));
		inputs.get(2).add("std::shared_ptr<rclcpp::Node>");
		inputs.get(2).add("std::shared_ptr<SubscriptionT>");
		
//		ArrayList<ArrayList<String>> dontUse = new ArrayList<ArrayList<String>>();
//		dontUse.add(new ArrayList<String>());
//		dontUse.get(0).addAll(blocks.get(1));
//		dontUse.get(0).addAll(blocks.get(2));
//		dontUse.add(new ArrayList<String>());
//		dontUse.get(1).addAll(blocks.get(0));
//		dontUse.get(1).addAll(blocks.get(2));
//		dontUse.add(new ArrayList<String>());
//		dontUse.get(2).addAll(blocks.get(0));
//		dontUse.get(2).addAll(blocks.get(1));
//		
//		for (ArrayList<String> dont : dontUse) {
//			dont.addAll(neverUse);
//		}
		
//		input.add("std::string");
//		input.add("std::string");
//		input.add("const rclcpp::QoS&");
		
		ArrayList<ArrayList<CppCode>> snippets = new ArrayList<ArrayList<CppCode>>();
		int index = 0;
		System.out.println("Blocks:");
		
		// We don't really want to do this until we have the actual synthesis code that is directed by things
		for (int i = 0; i < blocks.size(); i++) {
			List<List<String>> k = new ArrayList<>();
			k.add(new ArrayList<String>());
			for (int j = 0 ; j < blocks.get(i).size(); j++) {
				k.get(0).addAll(blockByRos1Name.get(blocks.get(i).get(j)));
			}
			ArrayList<String> dontUse = new ArrayList<String>();
			for (int j = 0; j < blocks.size(); j++) {
				if (i != j) {
					for (int l = 0; l < blocks.get(j).size(); l++) {
						dontUse.addAll(blockByRos1Name.get(blocks.get(j).get(l)));
					}
				}
			}
			
			dontUse.addAll(neverUse);
			
			System.out.print("Block: ");
			System.out.println(k);
//			k.add(blocks.get(i));
			System.out.println(dontUse);
			
			ArrayList<ArrayList<String>> strss = Synthesis.synthesizeAll(pn, inputs.get(i), 3, k, dontUse);
			System.out.print("Number of possibilities generated for block ");
			
			
			String blocksSubstring = k.toString().substring(0, Math.min(k.toString().length(), 54));
			blocksSubstring = blocksSubstring + ((blocksSubstring.length() < k.toString().length()) ? "...]" : "");
			System.out.print(blocksSubstring);
			System.out.print(": ");
			System.out.println(strss.size());
			
			int maxShown = 280;
			
			System.out.println("The top " + Integer.toString(maxShown) + ":");
			snippets.add(new ArrayList<CppCode>());
			for (ArrayList<String> strs : strss.subList(0, Math.min(strss.size(), maxShown))) {
				System.out.println(strs);
				snippets.get(index).add(new CppCode(mtpn, strs));
			}
			index++;
		}
		
		for (ArrayList<CppCode> snips : snippets) {
			System.out.print(snips);
		}
		
//		ArrayList<String> apiStrings = new ArrayList<String>();
//		apiStrings.add("rclcpp::init");
//		apiStrings.add("rclcpp::Node::make_shared");
//		apiStrings.add("rclcpp::Node::Shared_to_unshared");
//		apiStrings.add("rclcpp::Node::create_subscription");
//		apiStrings.add("rclcpp::spin");
////		apiStrings.add("rclcpp::shutdown");
//		
//		CppCode cpp = new CppCode(mtpn, apiStrings);
//		
//		
//		
//		
//		
//		
////		System.out.println(cpp.createCodeWithHoles());
//		
//		HashMap<String, Type> inputTypes = new HashMap<String, Type>();
//		
//		Type std_string = new Type("std::string");
//		
//		inputTypes.put("argc", new Type("int"));
//		inputTypes.put("argv", new Type("char *"));
//		inputTypes.put("node_name", std_string);
//		inputTypes.put("topic_name", std_string);
//		inputTypes.put("rmw_qos_profile_system_default", new Type("rclcpp::QoS"));
//		inputTypes.put("chatterCallback", new Type("CallbackT"));
//		
//		ArrayList<String> holeFillers = cpp.generateCodeWithInputs(inputTypes);
//		
//		// Get the sketch
//		String sketchContents = new String(Files.readAllBytes(Paths.get("ex1/sketches/listener.sketch")));
//		ArrayList<String> filledOutSketches = new ArrayList<String>();
//		for (String fill : holeFillers) {
//			String [] splits = fill.split("\n");
//			String replaceTag = "\\?\\?";
//			String sketch = sketchContents;
//			int i = 0;
//			while (sketch.indexOf("??") > -1 && i < splits.length) {
//				System.out.println("Replacing an instance of ?? in the replace string.");
//				sketch = sketch.replaceFirst(replaceTag, splits[i]);
//				i++;
//			}
//			filledOutSketches.add(sketch);
//		}
//		
//		int times = 0;
//		for (String sketch : filledOutSketches) {
//			System.out.println("the full sketch: ");
//			System.out.println(sketch);
//			
//			try {
//				FileWriter fw = new FileWriter("ex1/sketches/listener" + Integer.toString(times) + ".cpp");
//				
//				fw.write(sketch);
//				fw.flush();
//				fw.close();
//			} catch (Exception e) {
//				System.out.println(e);
//			}
//			times++;
//		}
						
//		System.out.println(cpp.generateCodeWithInputs(inputTypes));
		
		
		MethodsToPetriNet.createDotFile(pn);
		
	}
}
