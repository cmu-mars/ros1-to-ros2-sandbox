package ros2sy.code;

import java.util.ArrayList;
import java.util.HashMap;

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
	/**
	 * A list of the API calls that this sequence of C++ code represents
	 */
	ArrayList<Method> apis;
	ArrayList<Integer> numHolesPerApi;
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
		this.apis = new ArrayList<Method>();
		this.numHolesPerApi = new ArrayList<Integer>();
		this.holeTypes = new ArrayList<Type>();
		for (String a : apis) {
			if (mt.hasMethodWithNickname(a)) {
				Method m = mt.getMethodFromNickname(a);
				
				if (mt.isNotDummyMethod(m)) {
					this.apis.add(m);
					
					if (mt.isPointerFieldAccess(m)) {
						this.numHolesPerApi.add(0);
					} else {
						int instReq = (m.isClassMethod) ? 1 : 0;
						int numReq = m.numRequiredArgs();
						int numOpts = this.numberOptionals(a);
						
						this.numHolesPerApi.add(instReq + numReq + numOpts);
						
//						System.out.println("<" + Integer.toString(numOpts) + ", " + m.toString() + ">");
						
						if (m.isClassMethod) {
							this.holeTypes.add(m.fromClass);
							this.updateHoleTypeCounts(m.fromClass);
						}
						
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
		
		
//		System.out.println("Number of holes to fill:");
//		System.out.println(this.numHolesToFill);
		
		for (String key : typeCounts.keySet()) {
			System.out.println(key + ": " + typeCounts.get(key).toString());
		}
	}
	
	/**
	 * Produces the C++ code with holes that this CppCode object represents.
	 * 
	 * @return a string, containing the code
	 */
	public String createCodeWithHoles(InputVariables in) throws CodeGenerationException {
		String code = "";
		int holes = 0;
		boolean isPointerFieldAccess = false;
		for (int i = 0; i < this.apis.size(); i++) {
			Method m = this.apis.get(i);
			if (m.name.equals(MethodsToPetriNet.un_pointer_name)) {
				isPointerFieldAccess = true;
			} else {
				int numHolesOffset = 0;
				if (!m.returnType.getPlainType().equals("void")) {
					String id = this.getFreshId();
					this.results.put(id, m.returnType);
					in.addNewResult(id, m.returnType, i);
					code += m.returnType.toString() + " " + id + " = ";
				}
				if (m.isClassMethod) {
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
				code += m.name + "(";
				int numArgHoles = this.numHolesPerApi.get(i) - numHolesOffset;
				for (int j = 0; j < numArgHoles; j++) {
					code += "#" + Integer.toString(holes);
					if (j < (numArgHoles - 1)) {
						code += ", ";
					}
					holes++;
				}
				code += ");\n";
			}
			
			if (holes > this.numHolesToFill) {
				throw new CodeGenerationException("More holes were found for this code than were originally anticipated; expected " + Integer.toString(this.numHolesToFill) + " holes, but found at least " + Integer.toString(holes) + ":\n" + code + "\nand expected this many holes per API:\n" + this.holesPerApiString());
			}
		}
		
		return code;
	}
	
	public ArrayList<String> generateCodeWithInputs(HashMap<String, Type> inputs) throws CodeGenerationException {
		// Enumerates how many of each type we have, as well as contains convenience
		// methods for different ways of querying the inputs.
		InputVariables in = new InputVariables(inputs);
		String holeyCode = this.createCodeWithHoles(in);
		System.out.println("Hole-y code:");
		System.out.println(holeyCode);
		
		// I want to have some notion of what's available at certain points in
		// the code? Since we store all of the results of each API call
		// beforehand...
		int possibles = 1;
		ArrayList<Integer> holesLeft = new ArrayList<Integer>();
		for (int i = 0; i < this.holeTypes.size(); i++) {
			Type t = this.holeTypes.get(i);
			String plain = t.getPlainType();
			String holeTag = "#" + Integer.toString(i);
			
			if (!in.hasTypeCountsForString(plain)) {
				System.out.println("Cannot find key <" + plain + "> as one of the inputs!");
				int lineNumber = this.getLineNumberByHoleNumber(i);
				ArrayList<String> possibleNames = in.resultsOfTypeAvailable(lineNumber, t);
				System.out.print("The line number computed: ");
				System.out.println(lineNumber);
				if (possibleNames.size() > 0) {
					possibles *= possibleNames.size();
					if (possibleNames.size() == 1) {
						System.out.println("We have exactly one possible name!");
						
						holeyCode = holeyCode.replaceFirst(holeTag, possibleNames.get(0));
					}
				} else {
					System.out.println("No possible names found for type " + plain);
				}
			} else {
				Integer numPossibleHoleFillers = in.numTypesForString(plain);
				System.out.println(holeTag + ": " + numPossibleHoleFillers.toString() + " possibilities");
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
		System.out.println("There are " + Integer.toString(possibles) + " possiblities in total.");
		
		// Don't do this if there are a ton of possibilities -- worst case exponential
		// in time, and maybe space too.
		ArrayList<String> possibleFills = fillRemainingHoles(holeyCode, in, holesLeft);
		System.out.println("We have " + Integer.toString(possibleFills.size()) + " possible ways of filling: ");
		for (String fill : possibleFills) {
			System.out.println("A possible way of filling the holes:\n\n" + fill + "\n");
		}
		
		
		// Should hopefully be filled by now
//		return holeyCode;
		return possibleFills;
	}
	
	public ArrayList<String> getResultTypeNames() {
		ArrayList<String> resultTypes = new ArrayList<String>();
		
		for (String key : this.results.keySet()) {
			resultTypes.add(this.results.get(key).toString());
		}
		
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
		
	private ArrayList<String> fillRemainingHoles(String holeyCode, InputVariables in, ArrayList<Integer> remainingHoleIndices) {
		ArrayList<String> possibleFills = new ArrayList<String>();
		System.out.println(remainingHoleIndices.size());
		if (remainingHoleIndices.size() == 0) {
			possibleFills.add(holeyCode);
			return possibleFills;
		}
		
		int first = remainingHoleIndices.get(0);
		System.out.println(first);
		
		String holeTag = "#" + Integer.toString(first);
		ArrayList<Integer> rest = new ArrayList<Integer>(remainingHoleIndices.subList(1, remainingHoleIndices.size()));
		
		String holeTypeName = this.holeTypes.get(first).getPlainType();
		System.out.println(holeTypeName);
		System.out.println(in.hasNamesForString(holeTypeName));
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
		String tipe = t.getPlainType();
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
		
		nameSub.replaceAll("[ALT-()]*", "");
		// Add one because it's actually zero-indexed
		return Integer.valueOf(nameSub).intValue() + 1;
	}
}
