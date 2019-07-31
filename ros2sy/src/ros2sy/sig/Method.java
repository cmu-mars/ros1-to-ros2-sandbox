package ros2sy.sig;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import ros2sy.exception.ArgParseException;
import ros2sy.petri.MethodsToPetriNet;

/**
 * Represents a Method from the APIs that you want to represent.
 * 
 * @author audrey
 *
 */
public class Method {
	private static final Logger LOGGER = LogManager.getLogger(Method.class.getName());
	public boolean isConstructor;
	public String name;
	public ArrayList<Arg> args;
	public Type returnType;
	public boolean isClassMethod;
	public Type fromClass;
	public MethodType methodType;
	public String ros1Name = "";
	public HashSet<String> tags = new HashSet<>();
	public ArrayList<String> include = new ArrayList<>();
	public ArrayList<TemplateParameter> tparams = new ArrayList<>();
	private HashMap<String, TemplateParameter> templateMap = new HashMap<>();
	public boolean hasTemplateParamaters = false;
	private int numRequiredTemplateParams = 0;
	private String functionTypeString = "";
//	private int numRequiredTemplateParameters;
	
	private static String [] rclcpp_classes = {
		"Node", "Publisher<MessageT, Alloc>", "Subscription<CallbackMessageT, Alloc>", "GenericRate", "WallRate", "Rate"
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
			LOGGER.trace("The method <{}> has lowest index ({})", name, lowestIndex);
			if (lowestIndex != -1) {
				if (Method.rclcpp_classes[lowestIndex].equals("GenericRate")) {
					this.fromClass = new Type("rclcpp::" + Method.rclcpp_classes[lowestIndex] + "<Clock>");
				} else {
					this.fromClass = new Type("rclcpp::" + Method.rclcpp_classes[lowestIndex]);
				}
			} else {
				this.fromClass = new Type(this.name.substring(0, this.name.lastIndexOf("::")));
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
	 * @param functionType					either "constructor", "macro", or the empty
	 * 										string, used to determine what kind of
	 * 										method this is.
	 */
	public Method(String name, ArrayList<String> args, String returnType, String functionType) {
		this.functionTypeString = functionType;
		this.name = name;
		this.args = stringListToArgList(args);
		this.returnType = new Type(returnType);
		
		if (functionType.equals("constructor")) {
			LOGGER.trace("The method <{}> is a constructor type", name);
			this.methodType = MethodType.CONSTRUCTOR;
		} else if (functionType.equals("macro")) {
			LOGGER.trace("The method <{}> is a macro type", name);
			this.methodType = MethodType.MACRO;
		} else if (functionType.equals("member_access")) {
			LOGGER.info("The method <{}> is a member access type", name);
			this.methodType = MethodType.MEMBER_ACCESS;
			
			LOGGER.info("{}", this.methodType == MethodType.MEMBER_ACCESS);
		} else {
			this.methodType = Method.determineMethodType(name);
		}
		
		this.isClassMethod = this.methodType == MethodType.CLASS_METHOD;
		this.isConstructor = this.methodType == MethodType.CONSTRUCTOR;
		
		if (this.isClassMethod || this.isConstructor || this.methodType == MethodType.MEMBER_ACCESS) {
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
			} else {
				LOGGER.debug("Could not get fromClass for method {}", this.name);
				if (this.name.indexOf("std_msgs::msg::String") > -1) {
					this.fromClass = new Type("std_msgs::msg::String");
				}
			}
		}
	}
	
	public boolean isMemberAccess() {
		return this.methodType == MethodType.MEMBER_ACCESS;
	}
	
	public void addTemplateParameter(String name) {
		if (!this.hasTemplateParamaters) {
			this.hasTemplateParamaters = true;
		}
		this.numRequiredTemplateParams++;
		if (name.equals("MessageT")) {
			MessageT tp = new MessageT(name);
			
			this.templateMap.put("MessageT", tp);
			this.tparams.add(tp);
		} else {
			TemplateParameter tp = new TemplateParameter(name);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		}
	}
	
	public void addTemplateParameter(String name, boolean isClassTemplate) {
		if (!this.hasTemplateParamaters) {
			this.hasTemplateParamaters = true;
		}
		this.numRequiredTemplateParams++;
		if (name.equals("MessageT")) {
			MessageT tp = new MessageT(name, isClassTemplate);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		} else {
			TemplateParameter tp = new TemplateParameter(name, isClassTemplate);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		}
	}
	
	public void addTemplateParameter(String name, String value) {
		if (!this.hasTemplateParamaters) {
			this.hasTemplateParamaters = true;
		}
		if (name.equals("MessageT")) {
			MessageT tp = new MessageT(name, value);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		} else {
			TemplateParameter tp = new TemplateParameter(name, value);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		}
	}
	
	public void addTemplateParameter(String name, String value, boolean isClassTemplate) {
		if (!this.hasTemplateParamaters) {
			this.hasTemplateParamaters = true;
		}
		if (name.equals("MessageT")) {
			MessageT tp = new MessageT(name, value, isClassTemplate);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		} else {
			TemplateParameter tp = new TemplateParameter(name, value, isClassTemplate);
			this.templateMap.put(name, tp);
			this.tparams.add(tp);
		}
	}
	
	public boolean hasTemplateParameter(String name) {
		return this.templateMap.containsKey(name);
	}
	
	public TemplateParameter getTemplateParameter(String name) {
		return this.templateMap.get(name);
	}
	
	public boolean hasTemplateDefault(String name) {
		return this.templateMap.get(name).hasDefault;
	}
	
	public String getTemplateDefault(String name) {
		return this.templateMap.get(name).getDefaultValue();
	}
	
	public String apply(String ...givenArgs) {
		String functionString = "";
		
		int minArgs = this.numTemplateParametersRequired() + this.numInstancesRequired() + this.numRequiredArgs();
		int maxArgs = this.tparams.size() + this.numInstancesRequired() + this.numRequiredArgs() + this.numOptionalArgs();
		
		if (givenArgs.length < minArgs) {
			LOGGER.warn("COULD NOT APPLY TO METHOD {}: too few arguments given, need at least {} but was given {}", Integer.toString(this.numInstancesRequired() + this.numRequiredArgs()), Integer.toString(givenArgs.length));
		} else if (givenArgs.length > maxArgs) {
			LOGGER.warn("COULD NOT APPLY TO METHOD {}: too many arguments given, need at most {} but was given {}", Integer.toString(maxArgs), Integer.toString(givenArgs.length));
		} else {
			if (this.numInstancesRequired() > 0) {
				functionString = givenArgs[0] + "." + this.getBaseName() + "(";
			} else {
				functionString = this.name + "(";
			}
			
			for (int i = this.numInstancesRequired(); i < givenArgs.length; i++) {
				functionString = functionString + givenArgs[i];
				if (i < givenArgs.length - 1) {
					functionString = functionString + ", ";
				}
			}
			functionString = functionString + ")";
		}
		
		return functionString;
	}
	
	public ArrayList<TemplateParameter> getRequiredTemplateParameters() {
		ArrayList<TemplateParameter> params = new ArrayList<>();
		
		for (TemplateParameter tp : tparams) {
			if (!tp.hasDefault) {
				params.add(tp);
			}
		}
		
		return params;
	}
	
	public int numTemplateParametersRequired() {
		return this.numRequiredTemplateParams;
	}
	
	public int numTemplateParameters() {
		return this.tparams.size();
	}
	
	public int numInstancesRequired() {
		return (this.isClassMethod) ? 1 : 0;
	}
	
	public String getBaseName() {
		int lastIndex = this.name.lastIndexOf("::");
		if (lastIndex > -1) {
			return this.name.substring(lastIndex + 2);
		}
		return this.name;
	}
	
	public String getPrintingName() {
		if (this.isClassMethod || this.isMemberAccess()) {
			return this.getBaseName();
		}
		return this.name;
	}
	
	public void addInclude(String includeString) {
		this.include.add(includeString);
	}
	
	public ArrayList<String> getInclude() {
		return this.include;
	}
	
	public void addTag(String newTag) {
		this.tags.add(newTag);
	}
	
	public HashSet<String> getTags() {
		return this.tags;
	}
	
	public boolean requiresTemplateParams() {
		return this.hasTemplateParamaters && this.numRequiredTemplateParams > 0;
	}
	
	public boolean hasTag(String tagName) {
		return this.tags.contains(tagName);
	}
	
	public void setRos1Name(String name) {
		this.ros1Name = name;
	}
	
	public String getPrintableReturnType(HashMap<String, String> tempVarToType) {
		String returnString = this.returnType.toString();
		if (returnString.length() == 0 && this.isConstructor) {
			returnString = this.fromClass.toString();
		}
		returnString = this.replaceTypeParams(returnString);
		
		return Method.replaceTypeParamsInString(tempVarToType, returnString);
	}
	
	public String getPrintableArgType(HashMap<String, String> tempVarToType, int index) {
		if (index < this.args.size()) {
			String argTypeString = this.replaceTypeParams(this.args.get(index).argType);
			
			return Method.replaceTypeParamsInString(tempVarToType, argTypeString);
		}
		LOGGER.warn("Called getPrintableArgType with arg index of {}, but expected something less than {}", index, this.args.size());
		return "<ERROR: INDEX TOO BIG FOR ARGS ARRAY IN METHOD " + this.name + ">";
	}
	
	public String getPrintableClassType(HashMap<String, String> tempVarToType) {
		if (this.isClassMethod) {
			try {
				String classString = replaceTypeParams(this.fromClass);
				
				return Method.replaceTypeParamsInString(tempVarToType, classString);
			} catch (Exception e) {
				LOGGER.warn("Caught an exception: ", e);
			}
		}
		
		return "";
	}
	
	public String replaceTypeParams(Type t) {
		return replaceTypeParams(t.toString());
	}
	
	public String replaceTypeParams(String t) {
		String tString = t.toString();
		boolean changed = true;
		while (changed) {
			changed = false;
			for (TemplateParameter tp : tparams) {
				if (tp.hasDefault) {
					if(tString.indexOf(tp.name) > -1) {
//						LOGGER.info("Replacing {} with {} in {} in method {}", tp.name, tp.getDefaultValue(), tString, this.name);
						String newTString = tString.replaceAll("\\b" + tp.name + "\\b", tp.getDefaultValue());
						if (!newTString.equals(tString)) {
							changed = true;
							tString = newTString;
						}
					}
				}
			}
		}
		
		
//		if (tString.indexOf("Alloc") > -1 || t.indexOf("Alloc") > -1) {
//			LOGGER.info("{} -- Before vs After: {} vs {}", this.name, t, tString);
//		}
		
		return tString;
	}
	
	/**
	 * Gives the signature of the method, including its name, the types
	 * of its args, and its return type.
	 */
	@Override
	public String toString() {
		Object[] argsArray = this.args.toArray();
		String str = "";
		if (this.hasTemplateParamaters) {
			str = str + "<";
			
			for (int i = 0; i < this.tparams.size(); i++) {
				str = str + this.tparams.get(i).toString() + ((i < this.tparams.size() - 1) ? ", " : "");
			}
			
			str = str + ">";
		}
		
		str = str + "(" + (((Arg) argsArray[0]).toString());
		for (int i = 1; i < argsArray.length; i++) {
			str = str + ", " + (((Arg) argsArray[i]).toString());
		}
		str = str + ")";
		
		String returnString = this.returnType.toString();
		
		str = str + " -> " + ((this.isConstructor && returnString.length() == 0) ? this.fromClass.toString() : returnString);
		
		return this.name + ": " + str; //str + this.returnType.toString();
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
	
	public int getNumActualArgs(int numOptArgsUsing) {
		return this.numRequiredArgs() + Math.min(this.numOptionalArgs(), numOptArgsUsing);
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
	
	private ArrayList<String> argsListToStringList() {
		ArrayList<String> strings = new ArrayList<String>();
		
		for (Arg a : this.args) {
			strings.add(a.toString());
		}
		
		return strings;
	}
	
	public static String replaceTypeParamsInString(HashMap<String, String> typeVarToType, String tipe) {
		for (Map.Entry<String, String> entry : typeVarToType.entrySet()) {
			tipe = tipe.replaceAll(entry.getKey(), entry.getValue());
		}
		
		return tipe;
	}
	
	public static String replaceTypeParamsInString(MethodsToPetriNet mtpn, String tipe) {
		return mtpn.replaceTypeVars(tipe);
	}
	
	public static Type replaceTypeParamsInType(MethodsToPetriNet mtpn, Type t) {
		return new Type(mtpn.replaceTypeVars(t.toString()));
	}
	
	public static Type replaceTypeParamsInType(HashMap<String, String> typeVarToType, Type t) {
		return new Type(replaceTypeParamsInString(typeVarToType, t.toString()));
	}
	
	public Method replaceParametricTypeVariables(MethodsToPetriNet mtpn) {
		String newName = this.replaceTypeParams(this.name);
		String returnTypeString = this.replaceTypeParams(this.returnType);
		ArrayList<String> newArgs = this.argsListToStringList();
		
		newName = mtpn.replaceTypeVars(newName);
//		if (this.name.indexOf("rclcpp::Node::create_publisher") > -1) {
//			LOGGER.info("Return before: {}", returnTypeString);
//		}
		returnTypeString = mtpn.replaceTypeVars(returnTypeString);
//		if (this.name.indexOf("rclcpp::Node::create_publisher") > -1) {
//			LOGGER.info("Return after: {}", returnTypeString);
//		}
		for (int i = 0; i < newArgs.size(); i++) {
			newArgs.set(i, mtpn.replaceTypeVars(newArgs.get(i)));
		}
		
		Method newMethod = (this.functionTypeString.length() > 0) ? new Method(newName, newArgs, returnTypeString, functionTypeString) : new Method(newName, newArgs, returnTypeString);
		configureNewMethod(newMethod);
		if (newMethod.isClassMethod || newMethod.isMemberAccess()) {
			newMethod.fromClass = new Type(mtpn.replaceTypeVars(this.replaceTypeParams(this.fromClass)));
		}
		
		for (TemplateParameter tp : this.tparams) {
			if (tp.hasDefault) {
				newMethod.addTemplateParameter(tp.name, this.replaceTypeParams(tp.getDefaultValue()), tp.isClassTemplate());
			} else if (mtpn.containsTypeVarKey(tp.name)) {
				newMethod.addTemplateParameter(tp.name, this.replaceTypeParams(mtpn.getTypeVarReplacement(tp.name)), tp.isClassTemplate());
			} else {
				newMethod.addTemplateParameter(tp.name, tp.isClassTemplate());
			}
		}
		return newMethod;
	}
	
	private void configureNewMethod(Method newMethod) {
		newMethod.methodType = this.methodType;
		newMethod.isClassMethod = this.isClassMethod;
		newMethod.isConstructor = this.isConstructor;
	}
	
	public Method replaceParametricTypeVariables(HashMap<String, String> typeVarToType) {
		String newName = this.replaceTypeParams(this.name);
		String returnTypeString = this.replaceTypeParams(this.returnType);
		ArrayList<String> newArgs = this.argsListToStringList();
		
		for (int i = 0; i < newArgs.size(); i++) {
			newArgs.set(i, this.replaceTypeParams(newArgs.get(i).toString()));
		}
		
		for (Map.Entry<String, String> entry : typeVarToType.entrySet()) {
			newName = newName.replaceAll(entry.getKey(), entry.getValue());
			returnTypeString = returnTypeString.replaceAll(entry.getKey(), entry.getValue());
			for (int i = 0; i < newArgs.size(); i++) {
				newArgs.set(i, newArgs.get(i).replaceAll(entry.getKey(), entry.getValue()));
			}
		}
		
		Method newMethod = (this.functionTypeString.length() > 0) ? new Method(newName, newArgs, returnTypeString, functionTypeString) : new Method(newName, newArgs, returnTypeString);
		configureNewMethod(newMethod);
		if (newMethod.isClassMethod || newMethod.isMemberAccess()) {
			newMethod.fromClass = Method.replaceTypeParamsInType(typeVarToType, this.fromClass);
		}
		
		
		for (TemplateParameter tp : this.tparams) {
			if (!typeVarToType.containsKey(tp.name)) {
				if (tp.hasDefault) {
					newMethod.addTemplateParameter(tp.name, this.replaceTypeParams(tp.getDefaultValue()), tp.isClassTemplate());
				} else {
					newMethod.addTemplateParameter(tp.name, tp.isClassTemplate());
				}
			} else {
				newMethod.addTemplateParameter(tp.name, typeVarToType.get(tp.name), tp.isClassTemplate());
			}
		}
		
		return newMethod;
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
	
	public static Set<String> getAllRequiredTemplateParameters(ArrayList<Method> methods) {
		HashSet<String> params = new HashSet<>();
		
		for (Method m : methods) {
			if (m.requiresTemplateParams()) {
				LOGGER.trace("The method {} requires {} template parameters", m.name, m.numRequiredTemplateParams);
				for (TemplateParameter p : m.getRequiredTemplateParameters()) {
					if (!params.contains(p.name)) {
//						LOGGER.info("Found a template parameter with name <{}>",  p.name);
						params.add(p.name);
					}
				}
			}
		}
		
		return params;
	}
	
	public static HashMap<String, HashSet<String>> methodSetsToStringSets(HashMap<String, HashSet<Method>> tagsMap, MethodsToPetriNet mtpn) {
		HashMap<String, HashSet<String>> namesByTag = new HashMap<>();
		
		for (Map.Entry<String, HashSet<Method>> entry : tagsMap.entrySet()) {
			namesByTag.put(entry.getKey(), new HashSet<String>());
			for (Method m : entry.getValue()) {
//				LOGGER.info("method.name={}, tag={}, names={}, nicknames={}", m.name,  entry.getKey(), namesByTag.get(entry.getKey()), mtpn.getNicknamesOfMethod(m));
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
				LOGGER.trace("Method with name <{}> is a class method", name);
				return MethodType.CLASS_METHOD;
			} else {
				LOGGER.trace("Method with name <{}> is an RCLCPP function", name);
				return MethodType.RCLCPP_FUNCTION;
			}
		} else if (name.startsWith("std::")) {
			LOGGER.trace("The method with name <{}> is a class method.", name);
			return MethodType.CLASS_METHOD;
		}
		LOGGER.trace("The method with name <{}> in just a default method type.", name);
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
