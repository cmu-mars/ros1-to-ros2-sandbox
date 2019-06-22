package ros2sy.code;

import java.util.ArrayList;
import java.util.HashMap;

import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.*;

public class CppCode {
	ArrayList<Method> apis;
	ArrayList<Integer> numHolesPerApi;
	HashMap<String, Integer> typeCounts = new HashMap<String, Integer>();
	public int numHolesToFill;
	ArrayList<Type> holeTypes;
	
	
	private int fresh_id = -1;
	public CppCode(MethodsToPetriNet mt, ArrayList<String> apis) {
		this.apis = new ArrayList<Method>();
		this.numHolesPerApi = new ArrayList<Integer>();
		this.holeTypes = new ArrayList<Type>();
		for (String a : apis) {
			if (mt.hasMethodNickname(a)) {
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
							this.updateTypeCounts(m.fromClass);
						}
						
						for (Arg arg : m.getMandatoryArgs()) {
							holeTypes.add(arg.argType);
							
							this.updateTypeCounts(arg.argType);
						}
						
						for (Arg arg : m.getOptionalArgs().subList(0, numOpts)) {
							holeTypes.add(arg.argType);
							this.updateTypeCounts(arg.argType);
						}
					}
				}
			}
		}
		
		this.numHolesToFill = 0;
		for (Integer i : this.numHolesPerApi) {
			this.numHolesToFill += i;
		}
		
		
//		System.out.println("Number of holes to fill:");
//		System.out.println(this.numHolesToFill);
		
		for (String key : typeCounts.keySet()) {
			System.out.println(key + ": " + typeCounts.get(key).toString());
		}
	}
	
	public String createCodeWithHoles() {
		String code = "";
		int holes = 0;
		boolean isPointerFieldAccess = false;
		for (int i = 0; i < this.apis.size(); i++) {
			Method m = this.apis.get(i);
			if (m.name.equals(MethodsToPetriNet.un_pointer_name)) {
				isPointerFieldAccess = true;
			} else {				
				if (!m.returnType.getPlainType().equals("void")) {
					String id = this.getFreshId();
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
				}
				code += m.name + "(";
				int numArgHoles = this.numHolesPerApi.get(i);
				for (int j = 0; j < numArgHoles; j++) {
					code += "#" + Integer.toString(holes);
					if (j < (numArgHoles - 1)) {
						code += ", ";
					}
					holes++;
				}
				code += ");\n";
			}
		}
		
		return code;
	}
	
	private String getFreshId() {
		this.fresh_id++;
		return "the_freshest_id_" + Integer.toString(this.fresh_id);
	}
	
	private void updateTypeCounts(Type t) {
		String tipe = t.getPlainType();
		if (!typeCounts.containsKey(tipe)) {
			typeCounts.put(tipe,  0);
		}
		int old = typeCounts.get(tipe);
		typeCounts.put(tipe, old + 1);
	}
	
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
