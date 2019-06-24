package ros2sy.code;

import java.util.ArrayList;
import java.util.HashMap;

import ros2sy.sig.*;

public class InputVariables {
	private HashMap<String, Integer> typeCounts = new HashMap<String, Integer>();
	private HashMap<String, Type> plainTypeToType = new HashMap<String, Type>();
	private HashMap<Type, ArrayList<String>> namesByType = new HashMap<Type, ArrayList<String>>();
	private HashMap<String, Type> inputs;
	
	public InputVariables(HashMap<String, Type> inputs) {
		this.inputs = inputs;
		
		for (String inputName : inputs.keySet()) {
			Type tipe = inputs.get(inputName);
			String t = tipe.getPlainType();
			if (!plainTypeToType.containsKey(t)) {
				plainTypeToType.put(t, tipe);
			}
			if (!typeCounts.containsKey(t)) {
				typeCounts.put(t, 0);
				namesByType.put(tipe, new ArrayList<String>());
			}
			int old = typeCounts.get(t);
			typeCounts.put(t,  old + 1);
			namesByType.get(tipe).add(inputName);
		}
	}
	
	public boolean contains(String typeName) {
		return hasTypeCountsForString(typeName) && hasTypeForPlain(typeName) && hasNamesForString(typeName);
	}
	
	public boolean hasTypeCountsForString(String typeName) {
		return this.typeCounts.containsKey(typeName);
	}
	
	public int numTypesForString(String str) {
		if (!hasTypeCountsForString(str)) {
			return 0;
		}
		return this.typeCounts.get(str);
	}
	
	private boolean hasTypeForPlain(String plain) {
		return this.plainTypeToType.containsKey(plain);
	}
	
	public Type typeForString(String plainTypeName) {
		return this.plainTypeToType.get(plainTypeName);
	}
	
	public boolean hasNamesForString(String typeName) {
		return hasTypeForPlain(typeName) && namesByType.containsKey(typeForString(typeName));
	}
	
	public ArrayList<String> getNamesForString(String typeName) {
		return namesByType.get(typeForString(typeName));
	}
	
	public String getName(String typeName, int index) {
		return this.getNamesForString(typeName).get(index);			
	}
}
