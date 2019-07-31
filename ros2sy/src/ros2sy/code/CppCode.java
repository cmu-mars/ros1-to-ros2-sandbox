package ros2sy.code;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import ros2sy.exception.CodeGenerationException;
import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.*;

/**
 * Holds a representation of C++ ROS code, and can be used to obtain a C++
 * program that typechecks.
 * 
 * This contains methods and fields that help to encode useful information
 * needed for turning a sequence of methods into actual compilable code.
 * 
 * @author audrey
 *
 */
public class CppCode {
	private static final Logger LOGGER = LogManager.getLogger(CppCode.class.getName());
	/**
	 * A list of the API calls that this sequence of C++ code represents
	 */
	ArrayList<Method> apis;
	ArrayList<Method> originalApis;
	ArrayList<Integer> numHolesPerApi;
	ArrayList<Integer> numTypeVarHolesPerApi;
	
	HashMap<String, Integer> typeCounts = new HashMap<String, Integer>();
	public int numHolesToFill;
	ArrayList<Type> holeTypes;
	
	public HashMap<String, Type> results = new HashMap<String, Type>();
	
	
	private int fresh_id = -1;
	
	
	/**
	 * Constructor for the CppCode class.
	 * 
	 * @param mt			the MethodsToPetriNet object, which will be used to convert
	 * 							the list of strings back into Method objects
	 * @param apis		a sequence of names of API's, generated by the program
	 * 							synthesizer using mt's PetriNet
	 */
	public CppCode(MethodsToPetriNet mt, ArrayList<String> apis) {
		LOGGER.debug("The full sequence of APIs:");
		int count = 0;
		for (String api : apis) {
			count++;
			LOGGER.debug("{}. {}", count, api);
		}
		
		this.apis = new ArrayList<Method>();
		this.originalApis = new ArrayList<Method>();
		this.numHolesPerApi = new ArrayList<Integer>();
		this.numTypeVarHolesPerApi = new ArrayList<>();
		this.holeTypes = new ArrayList<Type>();
		for (String a : apis) {
			if (mt.hasMethodWithNickname(a)) {
				Method m = mt.getMethodFromNickname(a);
				
				if (mt.isNotDummyMethod(m)) {
					this.originalApis.add(m);
					LOGGER.info("Number of required template parameters: ({}) -- {}/{}", m.name, m.numTemplateParametersRequired(), m.numTemplateParameters());
					
					if (mt.isPointerFieldAccess(m)) {
						this.apis.add(m);
						this.numTypeVarHolesPerApi.add(0);
						this.numHolesPerApi.add(0);
					} else {
						m = m.replaceParametricTypeVariables(mt);
						this.apis.add(m);
						int instReq = (m.isClassMethod || m.isMemberAccess()) ? 1 : 0;
						int numReq = m.numRequiredArgs();
						int numOpts = this.numberOptionals(a);
						
						this.numHolesPerApi.add(instReq + numReq + numOpts);
						this.numTypeVarHolesPerApi.add(m.numTemplateParametersRequired());
						
						LOGGER.info("<" + Integer.toString(this.apis.size()) + ", " + m.toString() + ">");
						
						if (m.isClassMethod || m.isMemberAccess()) {
							this.holeTypes.add(m.fromClass);
							this.updateHoleTypeCounts(m.fromClass);
						}
//						if (m.methodType == MethodType.MEMBER_ACCESS) {
//							this.holeTypes.add()
//						}
						
						for (Arg arg : m.getActualArgs(numOpts)) {
							holeTypes.add(arg.argType);
							
							this.updateHoleTypeCounts(arg.argType);
						}
						
//						for (Arg arg : m.getOptionalArgs().subList(0, numOpts)) {
//							holeTypes.add(arg.argType);
//							this.updateHoleTypeCounts(arg.argType);
//						}
					}
				}
			}
		}
		
		this.numHolesToFill = 0;
		for (Integer i : this.numHolesPerApi) {
			this.numHolesToFill += i.intValue();
		}
		
		
		LOGGER.info("Number of holes to fill:");
		LOGGER.info(this.numHolesToFill);
		
		for (String key : typeCounts.keySet()) {
			LOGGER.info(key + ": " + typeCounts.get(key).toString());
		}
	}
	
	/**
	 * Produces the C++ code with holes that this CppCode object represents.
	 * 
	 * @param in		an InputVariables object representing the inputs that we have
	 * 						been given. This is used for keeping track of results from
	 * 						calling APIs
	 * @return 		a string, containing the code with #0, #1, etc. for the holes
	 * @throws CodeGenerationException
	 * 						thrown when the number of holes expected does not match the
	 * 						number of holes found for this code
	 */
	public String createCodeWithHoles(InputVariables in, ArrayList<HoleType> apiHoleTypes) throws CodeGenerationException {
		LOGGER.traceEntry("createCodeWithHoles({})", in);
		
		String code = "";
		int holes = 0;
		boolean isPointerFieldAccess = false;
		
		LOGGER.info("ApiHoleTypes: {}", apiHoleTypes.size());
		int line = 0;
		for (int i = 0; i < this.apis.size(); i++) {
			Method m = this.apis.get(i);
			
			if (originalApis.get(i).hasTag("duration")) {
				LOGGER.info("The tags of method {}: {}", m.name, originalApis.get(i).tags);
			}
			
			if (m.name.equals(MethodsToPetriNet.un_pointer_name)) {
				LOGGER.info("We have a pointer dereference: {}", m.name);
				isPointerFieldAccess = true;
			} else {
				LOGGER.info("We don't have a pointer dereference: {}", m.name);
				int numHolesOffset = 0;
				if (!m.returnType.getPlainName().equals("void") && !m.returnType.toString().equals("") && !m.returnType.isVirtual() && !m.isConstructor) {
					if (line >= apiHoleTypes.size() || (line < apiHoleTypes.size() && apiHoleTypes.get(line) == HoleType.STATEMENT)) {
						String id = this.getFreshId();
						this.results.put(id, m.returnType);
//					LOGGER.info("Adding result type {} for hole {}", m.returnType, i);
						in.addNewResult(id, m.returnType, i);
//						code += m.returnType.toString() + " " + id + " = ";
						code += "auto " + id + " = ";
					}
				} else if (m.isConstructor && (line >= apiHoleTypes.size() || (apiHoleTypes.get(line) == HoleType.STATEMENT))) {
//					if (line < apiHoleTypes.size()) {
//						switch(apiHoleTypes.get(line)) {
//							case STATEMENT:
//								String id = this.getFreshId();
//
//
//								String resultString = (m.returnType.toString().equals("")) ? m.fromClass.toString() : m.returnType.toString();
//								Type resultType = (resultString.equals(m.returnType.toString())) ? m.returnType : m.fromClass;
//
//								code += resultString + " " + id;
//
//								this.results.put(id,  resultType);
//								in.addNewResult(id, resultType, i);
//								break;
//							case EXPRESSION:
//								break;
//						}
//					} else {
//
//					}
					
					LOGGER.warn("{} is bigger than size of api hole types: {}", line, apiHoleTypes.size());
					String id = this.getFreshId();
					
					
					String resultString = (m.returnType.toString().equals("")) ? m.fromClass.toString() : m.returnType.toString();
					Type resultType = (resultString.equals(m.returnType.toString())) ? m.returnType : m.fromClass;
					
					
					
					this.results.put(id,  resultType);
					in.addNewResult(id, resultType, i);
					
					
					
					Method original = originalApis.get(i);
					if (!original.hasTag("rate") && !original.hasTag("duration")) {
						code += "auto " + id + " = ";
//						code += " = ";
//						LOGGER.info("See the code now: {}", code);
					} else {
						code += resultString + " " + id;
					}
				}
				if (m.isClassMethod || m.isMemberAccess()) {
					code += "#" + Integer.toString(holes);
					
					if (isPointerFieldAccess) {
						code += "->";
						isPointerFieldAccess = false;
					} else {
						code += ".";
					}
					holes++;
					numHolesOffset = 1;
				}
				
				String lparen = (m.isMemberAccess()) ? "" : "(";
				String rparen = (m.isMemberAccess()) ? "" : ")";
				
				String templateParams = createTemplateParamString(m, originalApis.get(i));
				if (m.isConstructor && (originalApis.get(i).hasTag("rate") || originalApis.get(i).hasTag("duration"))) {
					code += templateParams; // + "(";
				} else {
					code += m.getPrintingName() + templateParams; // + "(";
				}
				code += lparen;
				int numArgHoles = this.numHolesPerApi.get(i) - numHolesOffset;
				for (int j = 0; j < numArgHoles; j++) {
					code += "#" + Integer.toString(holes);
					if (j < (numArgHoles - 1)) {
						code += ", ";
					}
					holes++;
				}
				
				code += rparen;
				
				if (!m.isMemberAccess()) {
					if (line >= apiHoleTypes.size() || apiHoleTypes.get(line) == HoleType.STATEMENT) {
						code += ";";
					}
				}
				code += "\n";
				
				
				line++;
			}
			
			if (holes > this.numHolesToFill) {
				LOGGER.warn("The number of holes in this code was exceeded, but expected exactly {} holes.", this.numHolesToFill);
				LOGGER.traceExit();
				throw new CodeGenerationException("More holes were found for this code than were originally anticipated; expected " + Integer.toString(this.numHolesToFill) + " holes, but found at least " + Integer.toString(holes) + ":\n" + code + "\nand expected this many holes per API:\n" + this.holesPerApiString());
			}
		}
		LOGGER.traceExit(code);
		return code;
	}
	
	private String createTemplateParamString(Method current, Method original) {
		String templateParams = "";
		if (original.requiresTemplateParams()) {
			templateParams = "";
			for (TemplateParameter param : original.getRequiredTemplateParameters()) {
				if (current.hasTemplateParameter(param.name) && current.hasTemplateDefault(param.name) && (!param.isClassTemplate())) {
					if (templateParams.length() > 0) {
						templateParams = templateParams + ", ";
					} else {
						templateParams = "<";
					}
					templateParams = templateParams + current.getTemplateDefault(param.name);
				}
			}
			if (templateParams.length() > 0) {
				templateParams += ">";
			}
		}
		return templateParams;
	}
	
	/**
	 * Creates C++ code and uses the given map of inputs as a way to fill args of
	 * functions for the code.
	 * 
	 * @param inputs			a map from strings (names of variables) to their types
	 * @return						a list of the possible code that can be generated with
	 * 									these inputs
	 * @throws CodeGenerationException
	 */
	public ArrayList<String> generateCodeWithInputs(HashMap<String, Type> inputs, ArrayList<HoleType> holes) throws CodeGenerationException {
		LOGGER.traceEntry("generateCodeWithInputs({})", inputs);
		// Enumerates how many of each type we have, as well as contains convenience
		// methods for different ways of querying the inputs.
		InputVariables in = new InputVariables(inputs);
		String holeyCode = this.createCodeWithHoles(in, holes);
		LOGGER.info("Hole-y code:\n{}", holeyCode);
//		LOGGER.info(holeyCode);
		
		// I want to have some notion of what's available at certain points in
		// the code? Since we store all of the results of each API call
		// beforehand...
		int possibles = 1;
		ArrayList<Integer> holesLeft = new ArrayList<Integer>();
		for (int i = 0; i < this.holeTypes.size(); i++) {
			Type t = this.holeTypes.get(i);
			String plain = t.getPlainName();
			String holeTag = "#" + Integer.toString(i);
			
			if (!in.hasTypeCountsForString(plain)) {
				LOGGER.info("{}: Cannot find key <{}> as one of the inputs!", holeTag, plain);
				int lineNumber = this.getLineNumberByHoleNumber(i);
				LOGGER.info("The line number computed: {}", lineNumber);

				ArrayList<String> possibleNames = in.resultsOfTypeAvailable(lineNumber, t);
				if (possibleNames.size() > 0) {
					possibles *= possibleNames.size();
					if (possibleNames.size() == 1) {
						LOGGER.trace("We have exactly one possible name!");
						
						holeyCode = holeyCode.replaceFirst(holeTag, possibleNames.get(0));
					}
				} else {
					LOGGER.info("No possible names found for type {}", plain);
					for (String inputName : inputs.keySet()) {
						Type inputType = inputs.get(inputName);
						
						LOGGER.trace("{},{}", t.valueTypeName, inputType.valueTypeName);
						LOGGER.trace(t.arrayLevel);
						LOGGER.trace(inputType.arrayLevel);
						LOGGER.trace(t.pointerLevel);
						LOGGER.trace(inputType.pointerLevel);
						if (t.asParamsEqual(inputType)) {
							LOGGER.trace("The type " + t.toString() + " is equivalent to the type " + inputType.toString());
							holeyCode = holeyCode.replaceFirst(holeTag, inputName);
							break;
						} else {
							LOGGER.trace("The type {} is NOT equivalent to the type {}", t.toString(), inputType.toString());
						}
					}
					
				}
			} else {
				Integer numPossibleHoleFillers = in.numTypesForString(plain);
				LOGGER.info("{}: {} possibilities for type {}", holeTag, numPossibleHoleFillers.toString(), plain);
				if (numPossibleHoleFillers > 0) {
					possibles *= numPossibleHoleFillers;
				}
				if (numPossibleHoleFillers == 1) {
					holeyCode = holeyCode.replaceFirst(holeTag, in.getName(plain, 0));
				} else if (numPossibleHoleFillers > 1) {
					holesLeft.add(i);
				}
			}
		}
		LOGGER.info("There are {} possibilities in total.", Integer.toString(possibles));
		
		// Don't do this if there are a ton of possibilities -- worst case exponential
		// in time, and maybe space too.
		ArrayList<String> possibleFills = fillRemainingHoles(holeyCode, in, holesLeft);
		LOGGER.info("We have {} possibile ways of filling", Integer.toString(possibleFills.size()));
		for (String fill : possibleFills) {
			LOGGER.debug("A possible way of filling the holes:\n{}", fill);
		}
		
		
		// Should hopefully be filled by now
//		return holeyCode;
		LOGGER.traceExit(possibleFills);
		return possibleFills;
	}
	
	public ArrayList<Method> getApis() {
		LOGGER.traceEntry();
		LOGGER.traceExit(this.apis);
		return this.apis;
	}
	
	public Set<String> getIncludes() {
		LOGGER.traceEntry();
		HashSet<String> includes = new HashSet<>();
		
		for (Method m : this.apis) {
			if (m.getInclude().size() > 0) {
				for (String includeString : m.getInclude()) {
					if (!includes.contains(includeString)) {						
						includes.add(includeString);
					}
				}
			}
		}
		
		LOGGER.traceExit(includes);
		return includes;
	}
	
	
	public ArrayList<String> getResultTypeNames() {
		LOGGER.traceEntry();
		ArrayList<String> resultTypes = new ArrayList<String>();
		
		for (String key : this.results.keySet()) {
			resultTypes.add(this.results.get(key).toString());
		}
		
		LOGGER.traceExit(resultTypes);
		return resultTypes;
	}
	
	public ArrayList<Type> getResultTypes() {
		ArrayList<Type> types = new ArrayList<Type>();
		
		for (String key: this.results.keySet()) {
			types.add(this.results.get(key));
		}
		
		return types;
	}
	
	private String holesPerApiString() {
		String descrip = "";
		
		for (int i = 0; i < this.apis.size(); i++) {
			descrip += apis.get(i).name + ": " + this.numHolesPerApi.get(i).toString() + "\n";
		}
		
		return descrip;
	}

	private int getLineNumberByHoleNumber(int holeNumber) {
		int lineNumber;
		int holes = 0;
		
		for (lineNumber = 0; lineNumber < this.numHolesPerApi.size(); lineNumber++) {
			holes += this.numHolesPerApi.get(lineNumber);
			if (holeNumber < holes) {
				break;
			}
		}
		
		return lineNumber;
	}
	
	/**
	 * Recursively fill in the rest of the holes.
	 * 
	 * This function is something like a factorial or exponential in time as
	 * a function of the number of remaining holes that need to be filled, so
	 * only use this function with very small numbers of remaining holes.
	 * 
	 * @param holeyCode							the String representation of the code thus far,
	 * 															which still has some hole tags to fill
	 * @param in											the input variables object which helps to keep
	 * 															track of available results
	 * @param remainingHoleIndices		a list of all of the hole indices that need
	 * 															filling
	 * @return												a list of all of the possible code snippets
	 * 															that can result from the inputs and code
	 * 															given
	 */
	private ArrayList<String> fillRemainingHoles(String holeyCode, InputVariables in, ArrayList<Integer> remainingHoleIndices) {
		ArrayList<String> possibleFills = new ArrayList<String>();
//		LOGGER.info(remainingHoleIndices.size());
		if (remainingHoleIndices.size() == 0) {
			possibleFills.add(holeyCode);
			return possibleFills;
		}
		
		int first = remainingHoleIndices.get(0);
		LOGGER.trace("First hole index: {}", first);
		
		String holeTag = "#" + Integer.toString(first);
		ArrayList<Integer> rest = new ArrayList<Integer>(remainingHoleIndices.subList(1, remainingHoleIndices.size()));
		
		String holeTypeName = this.holeTypes.get(first).getPlainName();
		LOGGER.trace("The name of the #{}'s type: {}", first, holeTypeName);
		LOGGER.trace("Do we have any results of type <{}>: {}", holeTypeName, in.hasNamesForString(holeTypeName));
		if (in.hasNamesForString(holeTypeName)) {
			for (String varName : in.getNamesForString(holeTypeName)) {
				possibleFills.addAll(fillRemainingHoles(holeyCode.replaceFirst(holeTag, varName), in, rest));
			}
		}
		return possibleFills;
	}
	
	/**
	 * Create a fresh variable name that we haven't used thus far.
	 * 
	 * @return the name
	 */
	private String getFreshId() {
		this.fresh_id++;
		return "the_freshest_id_" + Integer.toString(this.fresh_id);
	}
	
	/**
	 * Record/update the count of the given Type so that we can keep track of
	 * how many holes have each type
	 * 
	 * @param t	the type of a hole
	 */
	private void updateHoleTypeCounts(Type t) {
		String tipe = t.getPlainName();
		if (!typeCounts.containsKey(tipe)) {
			typeCounts.put(tipe,  0);
		}
		int old = typeCounts.get(tipe);
		typeCounts.put(tipe, old + 1);
	}
	
	/**
	 * Get the number of optional arguments that a method takes, given the
	 * method's string name from the PetriNet (i.e. the transition label's name.)
	 * 
	 * @param name		the id for the transition, which represents a method
	 * @return				the number of optional arguments
	 */
	private int numberOptionals(String name) {
		int index = name.indexOf("(ALT-");
		if (index == -1) {
			return 0;
		}
		String nameSub = name.substring(index, name.length());
		
		nameSub = nameSub.replaceAll("[-ALT\\(\\)]*", "");
		// Add one because it's actually zero-indexed
		return Integer.valueOf(nameSub).intValue() + 1;
	}
	
	
}
