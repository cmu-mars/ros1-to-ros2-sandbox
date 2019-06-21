package ros2sy.json;

import ros2sy.code.Method;

import java.util.*;


import java.io.FileReader;
import com.google.gson.JsonParser;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import ros2sy.petri.*;

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
				methods.addAll(ParseJson.unwrap("ros2", jo.get("ros2")));
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
		FileReader fr = new FileReader("ex1/listener-search-space.json");
		
		JsonParser jp = new JsonParser();
		
		try {
			JsonElement je = jp.parse(fr);
			
			if (je.isJsonObject()) {
				JsonObject jobj = je.getAsJsonObject();
				Set<String> keys = jobj.keySet();
				
				ArrayList<Method> methods = new ArrayList<Method>();
				
				for (String key : keys) {
					// this brings us to the code for each line
					methods.addAll(ParseJson.unwrap(key,  jobj.get(key)));
				}
				
//				System.out.println("Methods: ");
//				System.out.println(Integer.toString(methods.size()));
//				for (Method m : methods) {
//					System.out.println(m);
//				}
				
				PetriNet pn = MethodsToPetriNet.convert(methods);
				
				
				MethodsToPetriNet.createDotFile(pn);
				
			}
			
		} catch (JsonParseException e)  {
			System.out.println("Json parse had an exception.");
			System.out.println(e);
		}
		
	}
}
