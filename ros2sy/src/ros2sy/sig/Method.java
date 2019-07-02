package ros2sy.sig;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import ros2sy.exception.ArgParseException;
import ros2sy.petri.MethodsToPetriNet;

/**
 * Represents a Method from the APIs that you want to represent.
 * 
 * @author audrey
 *
 */
public class Method {
	public boolean isConstructor;
	public String name;
	public ArrayList<Arg> args;
	public Type returnType;
	public boolean isClassMethod;
	public Type fromClass;
	public MethodType methodType;
	public String instanceNeeded;
	public String ros1Name = "";
	public HashSet<String> tags = new HashSet<String>();
	private static String [] rclcpp_classes = {
		"Node", "Publisher", "Subscriber", "Rate"
	};
	
	/**
	 * Constructor for the Method class.
	 * 
	 * In this case, the method represented is assumed to not be a
	 * constructor.
	 * 
	 * @param name				a String, the name of the method
	 * @param args				an ArrayList<String> containing the types
	 * 										of all of the arguments to this method,
	 * 										and when the arg has an optional value, this
	 * 										is indicated after the type in parentheses
	 * 										like so: "<my-type> (=<my-default-value)"
	 * @param returnType	a String representing the return type of the
	 * 										method.
	 */
	public Method(String name, ArrayList<String> args, String returnType) {
		this.isConstructor = false;
		this.name = name;
		this.args = stringListToArgList(args);
		this.returnType = new Type(returnType);
		this.fromClass = new Type("void");
		this.methodType = Method.determineMethodType(name);
		this.isClassMethod = this.methodType == MethodType.CLASS_METHOD;
		
		if (this.isClassMethod) {
			int lowest = this.name.length();
			int lowestIndex = -1;
			for (int i = 0; i < Method.rclcpp_classes.length; i++) {
				int index = this.name.indexOf(Method.rclcpp_classes[i]);
				if (index > -1 && index < lowest) {
					lowest = index;
					lowestIndex = i;
				}
			}
			if (lowestIndex != -1) {
				this.fromClass = new Type("rclcpp::" + Method.rclcpp_classes[lowestIndex]);
			}
		}
	}
	/**
	 * Constructor for the Method class. This one allows you to pass in
	 * a String "type" that contains either "constructor" or "macro".
	 * 
	 * @param name					String, the name of the function
	 * @param args					the list of the args' types, with the optional
	 * 										values in parentheses like so:
	 * 										"<my-type> (=<my-default-value)"
	 * @param returnType		a String with the return type of the method
	 * @param type					either "constructor", "macro", or the empty
	 * 										string, used to determine what kind of
	 * 										method this is.
	 */
	public Method(String name, ArrayList<String> args, String returnType, String type) {
		this.name = name;
		this.args = stringListToArgList(args);
		this.returnType = new Type(returnType);
		
		if (type.equals("constructor")) {
			this.methodType = MethodType.CONSTRUCTOR;
		} else if (type.equals("macro")) {
			this.methodType = MethodType.MACRO;
		} else {
			this.methodType = Method.determineMethodType(name);
		}
		
		this.isClassMethod = this.methodType == MethodType.CLASS_METHOD;
		this.isConstructor = this.methodType == MethodType.CONSTRUCTOR;
		
		if (this.isClassMethod) {
			int lowest = this.name.length();
			int lowestIndex = -1;
			for (int i = 0; i < Method.rclcpp_classes.length; i++) {
				int index = this.name.indexOf(Method.rclcpp_classes[i]);
				if (index > -1 && index < lowest) {
					lowest = index;
					lowestIndex = i;
				}
			}
			if (lowestIndex != -1) {
				this.fromClass = new Type("rclcpp::" + Method.rclcpp_classes[lowestIndex]);
			}
		}
	}
	
	public void addTag(String newTag) {
		this.tags.add(newTag);
	}
	
	public HashSet<String> getTags() {
		return this.tags;
	}
	
	public boolean hasTag(String tagName) {
		return this.tags.contains(tagName);
	}
	
	public void setRos1Name(String name) {
		this.ros1Name = name;
	}
	
	
	/**
	 * Gives the signature of the method, including its name, the types
	 * of its args, and its return type.
	 */
	@Override
	public String toString() {
		Object[] argsArray = this.args.toArray();
		String str = "(" + (((Arg) argsArray[0]).toString());
		for (int i = 1; i < argsArray.length; i++) {
			str = str + ", " + (((Arg) argsArray[i]).toString());
		}
		str = str + ") -> ";
		return this.name + ": " + str + this.returnType.toString();
	}
	
	public int numArgs() {
		return this.args.size();
	}
	
	public int numOptionalArgs() {
		return this.getOptionalArgs().size();
	}
	
	public int numRequiredArgs() {
		int size = this.getMandatoryArgs().size();
		
		if (size == 1) {
			if (this.args.get(0).argType.valueTypeName.matches("void")) {
				return 0;
			}
		}
		
		return size;
	}
	
	/**
	 * Returns whether this method has any optional args.
	 * 
	 * @return boolean, true if there are optional args
	 */
	public boolean hasOptionalArgs() {		
		for (Arg a : this.args) { 
			if (a.hasDefault) {
				return true;
			}
		}
		return false;
	}
	
	public ArrayList<Arg> getActualArgs(int numOptArgsUsing) {
		int numReq = this.numRequiredArgs();
		if (numReq == 0 && !this.hasOptionalArgs()) {
			return new ArrayList<Arg>();
		}
		ArrayList<Arg> mandatory = new ArrayList<Arg>(this.getMandatoryArgs().subList(0, numReq));		
		
		ArrayList<Arg> optionals = this.getOptionalArgs();
		if (numOptArgsUsing >= this.numOptionalArgs()) {
			mandatory.addAll(optionals);
		} else {
			mandatory.addAll(optionals.subList(0, numOptArgsUsing));
		}
		return mandatory;
	}
	
	/**
	 * Get the list of all of the non-optional, i.e. mandatory
	 * args to this method.
	 * 
	 * @return 	an ArrayList of Arg objects, all of which are
	 * 				 	required.
	 */
	public ArrayList<Arg> getMandatoryArgs() {
		
		ArrayList<Arg> man = new ArrayList<Arg>();
		
		for (Arg a : this.args) {
			if (!a.hasDefault) {
				man.add(a);
			}
		}
		
		return man;
	}
	
	/**
	 * Get the list of all optional args to this method.
	 * 
	 * @return 	an ArrayList<Arg> object, which contains this
	 * 					method's optional args
	 */
	public ArrayList<Arg> getOptionalArgs() {
		ArrayList<Arg> opt = new ArrayList<Arg>();
		
		for (Arg a : this.args) {
			if (a.hasDefault) {
				opt.add(a);
			}
		}
		
		return opt;
	}
	
	public static HashMap<String, HashSet<String>> methodSetsToStringSets(HashMap<String, HashSet<Method>> map, MethodsToPetriNet mtpn) {
		HashMap<String, HashSet<String>> namesByTag = new HashMap<>();
		
		for (Map.Entry<String, HashSet<Method>> entry : map.entrySet()) {
			namesByTag.put(entry.getKey(), new HashSet<String>());
			for (Method m : entry.getValue()) {
				namesByTag.get(entry.getKey()).addAll(mtpn.getNicknamesOfMethod(m));
			}
		}
		
		return namesByTag;
	}
	
	/**
	 * Returns the specific MethodType for a method with a specific
	 * name.
	 * 
	 * This judges whether it is a class method by determining if
	 * the word 
	 * 
	 * @param name		the name of the method in question
	 * @return				a member of the MethodType enum that describes
	 * 							what kind of method it is
	 */
	
	private static MethodType determineMethodType(String name) {
		if (name.startsWith("rclcpp")) {
			if (Method.stringContains(name, rclcpp_classes)) {
				return MethodType.CLASS_METHOD;
			} else {
				return MethodType.RCLCPP_FUNCTION;
			}
		}
		return MethodType.DEFAULT_METHOD_TYPE;
	}
	
	/**
	 * 
	 * @param str
	 * @param tests
	 * @return
	 */
	private static boolean stringContains(String str, String[] tests) {
//		boolean contains = false;	
		for (String test : tests) {
			if (str.indexOf(test) > -1) {
				return true;
			}
//			contains = contains || (str.indexOf(test) > -1);
		}
		
		return false;
	}
	
	/**
	 * 
	 * @param strs
	 * @return
	 */
	
	private ArrayList<Arg> stringListToArgList(ArrayList<String> strs) {
		ArrayList<Arg> arglist = new ArrayList<Arg>();
		if (strs.size() > 0) {			
			for (String str : strs) {
				try {
					arglist.add(new Arg(str));
				} catch (ArgParseException e) {
					System.err.println("Could not parse arg " + str + " of method " + name);
					System.err.println(e);
				}
			}
		} else {
			try {
				arglist.add(new Arg("void"));
			} catch (ArgParseException e) {
				System.err.println("Could not parse void");
				e.printStackTrace();
			}
		}
		return arglist;
	}
}
