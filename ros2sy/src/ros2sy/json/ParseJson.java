package ros2sy.json;

import java.util.*;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

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
	private static final Logger LOGGER = LogManager.getLogger(ParseJson.class.getName());
	
	public ParseJson(String fileName) {
		
	}
	
	public static HashMap<String, String> getParameterMatchMap(String filename) {
		HashMap<String, String> paramMap = ParseJson.getDictionary(filename);
		
		return paramMap;
	}
	
	public static ArrayList<String> getAvailableTypes(String fileName) {
		ArrayList<String> tipes = new ArrayList<>();
		
		try {
			JsonObject jo = ParseJson.getJsonObjectFromFile(fileName);
			
			for (String key : jo.keySet()) {
				if (jo.get(key).isJsonObject()) {
					JsonObject value = jo.get(key).getAsJsonObject();
					if (value.has("types") && value.get("types").isJsonArray()) {
						for (JsonElement elt : value.get("types").getAsJsonArray()) {
							if (elt.isJsonPrimitive()) {
								tipes.add(elt.getAsString());
							}
						}
					}
				}
			}
		} catch (Exception e) {
			LOGGER.warn("Caught an exception:", e);
		}
		
		return tipes;
	}
	
	/**
	 * Finds the input variables map, given the name of a JSON file to parse the
	 * map from.
	 * 
	 * @param fileName		the name of the JSON file to parse
	 * @return						a HashMap of strings to strings, where the keys are the
	 * 									names of variables and the values are the type
	 */
	public static HashMap<String, String> getInputVariableToType(String fileName) {
		HashMap<String, String> varsToType = new HashMap<String, String>();
		
		String key = "input_var_to_type";
		
		try {
			JsonObject jo = ParseJson.getJsonObjectFromFile(fileName);
			
			if (jo.has(key)) {
				JsonElement inputVarToTypeMapMaybe = jo.get(key);
				
				if (inputVarToTypeMapMaybe.isJsonObject()) {
					JsonObject map = inputVarToTypeMapMaybe.getAsJsonObject();
					for (String k : map.keySet()) {
						if (map.get(k).isJsonPrimitive()) {
							varsToType.put(k,  map.get(k).getAsString());
						}
					}
				}
			}
		} catch (Exception e) {
			LOGGER.trace("Caught an exception in getInputVariableToType {}: {}", e.toString(), e);
		}
		
		return varsToType;
	}

	/**
	 * Reads in the supposed input types for each block from a JSON file. The
	 * object contained in the file must have a key "input_types" which has a
	 * value that is a nested array, a list of lists of strings.
	 * 
	 * @param fileName		the name of the JSON file to parse
	 * @return						the list of list of strings, where the strings are the
	 * 									names of types
	 */
	public static ArrayList<ArrayList<String>> getInputTypesFromFile(String fileName) {
		String key = "input_types";

		return ParseJson.getNestedArrayFromFile(fileName, key);
	}
	
	/**
	 * Retrieves the supposed correct answers from a JSON file
	 * 
	 * @param fileName		the name of the JSON file to parse
	 * @return						a list of lists of strings that contain the desired APIs
	 */
	public static ArrayList<ArrayList<String>> getCorrectAnswersFromFile(String fileName) {
		String key = "answers";
		
		return ParseJson.getNestedArrayFromFile(fileName, key);
	}
	
	public static HashMap<String, String> includesToReplace() {
		return ParseJson.getDictionary("ros1-to-ros2-pack-spec.json", "");
	}
	
	
	/**
	 * Gets a dictionary of strings to strings from a JSON file, and assumes
	 * that the whole JSON object in the file is the dictionary desired.
	 * 
	 * @param fileName		the name of the .json file to parse
	 * @return						a HashMap containing a dictionary
	 */
	public static HashMap<String, String> getDictionary(String fileName) {
		HashMap<String, String> includes = new HashMap<>();
		
		try {
			JsonObject jo = ParseJson.getJsonObjectFromFile(fileName);

			for (String key : jo.keySet()) {
				includes.put(key, jo.get(key).getAsString());
			}
		} catch (Exception e) {
			LOGGER.warn("Caught an exception while getting a dictionary: {}: {}", e.toString(), e);
		}
		
		return includes;
	}
	
	/**
	 * Gets a dictionary of strings to strings from a JSON file. It accesses the
	 * dictionary from the JSON object contained in the JSON file using the 
	 * optional key. If however the optional key is blank, it will simply just
	 * look at the entire JSON object in the file as the dictionary.
	 * 
	 * @param fileName			the name of the JSON file to parse
	 * @param optionalKey	a key, whose matching value in the JSON file is the
	 * 										desired dictionary
	 * @return							a HashMap representing the dictionary from the file
	 */
	public static HashMap<String, String> getDictionary(String fileName, String optionalKey) {
		HashMap<String, String> includes = new HashMap<>();
		
		try {
			JsonObject jo = ParseJson.getJsonObjectFromFile(fileName);
			if (!optionalKey.contentEquals("")) {
				if (jo.has(optionalKey)) {
					JsonObject dict = jo.get(optionalKey).getAsJsonObject();
					for (String key : dict.keySet()) {
						includes.put(key, dict.get(key).getAsString());
					}
				}
			} else {
				for (String key : jo.keySet()) {
					includes.put(key, jo.get(key).getAsString());
				}
			}
		} catch (Exception e) {
			LOGGER.warn("Caught an exception while getting a dictionary: {}: {}", e.toString(), e);
		}
		
		return includes;
	}
	
	/**
	 * Parses out a doubly nested array from a JSON file
	 * 
	 * @param fileName		the name of the JSON file that contains this doubly
	 * 									nested array
	 * @param key				the key whose value is a doubly nested array
	 * @return						an list of lists of strings that represents the doubly
	 * 									nested array
	 */
	public static ArrayList<ArrayList<String>> getNestedArrayFromFile(String fileName, String key) {
		ArrayList<ArrayList<String>> inputs = new ArrayList<>();
		try {
			JsonObject jo = ParseJson.getJsonObjectFromFile(fileName);
			
			if (jo.has(key)) {
				JsonElement inputArrayMaybe = jo.get(key);
				if (inputArrayMaybe.isJsonArray()) {
					int index = 0;
					for (JsonElement je : inputArrayMaybe.getAsJsonArray()) {
						inputs.add(new ArrayList<String>());
						if (je.isJsonArray()) {
							for (JsonElement arrayElmt : je.getAsJsonArray()) {
								if (arrayElmt.isJsonPrimitive()) {
									inputs.get(index).add(arrayElmt.getAsString());
								}
							}
						}
						index++;
					}
				}
			}
		} catch (Exception e) {
			LOGGER.warn("Caught an exception: {}: {}", e.toString(), e);
		}
		
		return inputs;
	}
	
	
	/**
	 * Returns the JSON object that represents a file
	 * 
	 * @param fileName			the name of the file to parse
	 * @return							a JSON object that represents that file
	 * @throws Exception
	 */
	private static JsonObject getJsonObjectFromFile(String fileName) throws Exception {
		FileReader fr = new FileReader(fileName);
		JsonParser jp = new JsonParser();
		JsonElement je = jp.parse(fr);
		if (!je.isJsonObject()) {
			LOGGER.warn("WARNING: this json element is not an object!");
//			System.out.println("WARNING: this json element is not an object!");
		}
		return je.getAsJsonObject();
	}
	
	/**
	 * Creates sets of methods for all tags in the tag file.
	 * 
	 * @param methods	a list of method objects that we wish to organize under
	 * 								particular tags
	 * @param tagFile	a JSON file that provides a dictionary for getting the
	 * 								tags for every method signature as well as the dictionary
	 * 								for getting the names of methods who have any given tag.
	 * @return					a HashMap with tags as keys and sets of methods as values
	 */
	public static HashMap<String, HashSet<Method>> tagMethods(ArrayList<Method> methods, String tagFile) {
		LOGGER.traceEntry();
		HashMap<String, HashSet<Method>> tagToMethods = new HashMap<>();
		
		try {
			JsonObject jo = getJsonObjectFromFile(tagFile);

			if (jo.has("sig_to_tags")) {
				JsonObject sig_to_tags = jo.get("sig_to_tags").getAsJsonObject();
				for (Method m : methods) {
					if (sig_to_tags.has(m.name) && sig_to_tags.get(m.name).isJsonArray()) {
						JsonElement methodTags = sig_to_tags.get(m.name);
						JsonArray tagsArray = methodTags.getAsJsonArray();
						for (JsonElement arrayElmt : tagsArray) {	
							if (arrayElmt.isJsonPrimitive()) {
								String str = arrayElmt.getAsString();

								m.addTag(str);
								LOGGER.trace("{}: {}", m.name, str);
							}
						}
					}
				}
			}

			if (jo.has("tag_to_sigs")) {
				JsonObject tag_to_sigs = jo.get("tag_to_sigs").getAsJsonObject();
				for (String key : tag_to_sigs.keySet()) {
					JsonArray namesArray = tag_to_sigs.get(key).getAsJsonArray();
					tagToMethods.put(key, new HashSet<Method>());
					HashSet<String> methodsNames = new HashSet<String>();
					for (JsonElement elmt : namesArray) {
						if (elmt.isJsonPrimitive()) {
							methodsNames.add(elmt.getAsString());
						}
					}
					if (methodsNames.size() > 0) {
						LOGGER.trace("size of methodsNames: {}", Integer.toString(methodsNames.size()));
						for (Method m : methods) {
							if (methodsNames.contains(m.name)) {
								tagToMethods.get(key).add(m);
							}
						}
					}
				}
			}
		} catch (Exception e) {
			LOGGER.warn("Caught exception: {} -- {}", e.toString(), e);
		}
		
		return tagToMethods;
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
		LOGGER.trace("key, is this a json object?: {}, {}", key, elmt.isJsonObject());
//		LOGGER.debug("is this a json object?: {}", elmt.isJsonObject());
		
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
				int index = methods.size();
				
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
				
				if ((jo.has("func_type") || jo.has("return")) && jo.has("include")) {
//					System.out.println("Setting include to " + jo.get("include").getAsString());
					JsonElement include = jo.get("include");
					if (include.isJsonPrimitive()) {
						methods.get(index).addInclude(include.getAsString());
					} else if (include.isJsonArray()) {
						for (JsonElement arryElmt : include.getAsJsonArray()) {		
							if (arryElmt.isJsonPrimitive()) {								
								methods.get(index).addInclude(arryElmt.getAsString());
							}
						}
					}
				}
				if (jo.has("template")) {
					LOGGER.trace("Method <{}> has template types",  methods.get(index).name);
					JsonArray jarray = jo.get("template").getAsJsonArray();
					
					for (JsonElement elm : jarray) {
						if (elm.isJsonObject()) {
							JsonObject temp = elm.getAsJsonObject();
							
							if (temp.has("name")) {
								String name = temp.get("name").getAsString();
								if (temp.has("value")) {
									String val = temp.get("value").getAsString();
									LOGGER.trace("Adding template parameter {}: {}", name, val);
									
									methods.get(index).addTemplateParameter(name, val);
								} else {
									
									LOGGER.trace("Adding template parameter {}", name);
									methods.get(index).addTemplateParameter(name);
								}
							}
						}
					}
				} else {
					LOGGER.trace("Method <{}> does not have template types", methods.get(index).name);
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
	 * Gets all of the methods and returns all of them in a single ArrayList
	 * 
	 * @param searchSpaceNames		a list of the "tags" that correspond to json
	 * 													files in the scrape_rclcpp_docs/jsons directory
	 * @return										a combined list of all of methods for each of
	 * 													those tags
	 */
	public static ArrayList<Method> getAllMethods(String ...searchSpaceNames) {
		ArrayList<Method> methods = new ArrayList<Method>();
		
		for (int i = 0; i < searchSpaceNames.length; i++) {
			String fileName = "scrape_rclcpp_docs/jsons/" + searchSpaceNames[i] + ".json";
			methods.addAll(ParseJson.parseOutMethods(fileName));
			LOGGER.info("Methods length: {}", methods.size());
		}
		
		return methods;
	}
	
	/**
	 * Take the example listener search space json file that I created, and
	 * then parse out the json, extracts the methods described as
	 * Method objects, and create a PetriNet from this.
	 * 
	 * The PetriNet is then output as a DOT file.
	 * 
	 * This is not usual main method.
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
		
		MethodsToPetriNet mtpn = new MethodsToPetriNet(methods, sillyToClone, new HashMap<String, HashSet<String>>());
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
		
		MethodsToPetriNet.createDotFile(pn);
	}
}
