package ros2sy.code;

import java.util.ArrayList;

public class Method {
	public boolean isConstructor;
	public String name;
	public ArrayList<Arg> args;
	public Type returnType;
	public boolean isClassMethod;
	public Type fromClass;
	public MethodType methodType;
	
	private static String [] rclcpp_classes = {
		"Node", "Publisher", "Subscriber", "Rate"
	};	
	
	public Method(String name, ArrayList<String> args, String returnType) {
		this.isConstructor = false;
		this.name = name;
		this.args = stringListToArgList(args);
		this.returnType = new Type(returnType);
		this.fromClass = new Type("void");
		this.methodType = Method.determineMethodType(name);
		this.isClassMethod = this.methodType == MethodType.CLASS_METHOD;
	}
	
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
	}
	
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
	
	private ArrayList<Arg> stringListToArgList(ArrayList<String> strs) {
		ArrayList<Arg> arglist = new ArrayList<Arg>();
		
		for (String str : strs) {
			try {
				arglist.add(new Arg(str));
			} catch (ArgParseException e) {
				System.err.println("Could not parse arg " + str + " of method " + name);
				System.err.println(e);
			}
		}
		return arglist;
	}
	
	
	private static boolean stringContains(String str, String[] tests) {
		boolean contains = false;	
		for (String test : tests) {
			contains = contains || (str.indexOf(test) > -1);
		}
		
		return contains;
	}
	
	public String toString() {
		Object[] argsArray = this.args.toArray();
		String str = "(" + (((Arg) argsArray[0]).toString());
		for (int i = 1; i < argsArray.length; i++) {
			str = str + ", " + (((Arg) argsArray[i]).toString());
		}
		str = str + ") -> ";
		return this.name + ": " + str + this.returnType.toString();
	}
	
	
	public boolean hasOptionalArgs() {		
		for (Arg a : this.args) { 
			if (a.hasDefault) {
				return true;
			}
		}
		
		return false;
	}
	
	public ArrayList<Arg> getMandatoryArgs() {
		ArrayList<Arg> man = new ArrayList<Arg>();
		
		for (Arg a : this.args) {
			if (!a.hasDefault) {
				man.add(a);
			}
		}
		
		return man;
	}
	
	public ArrayList<Arg> getOptionalArgs() {
		ArrayList<Arg> opt = new ArrayList<Arg>();
		
		for (Arg a : this.args) {
			if (a.hasDefault) {
				opt.add(a);
			}
		}
		
		return opt;
	}
}
