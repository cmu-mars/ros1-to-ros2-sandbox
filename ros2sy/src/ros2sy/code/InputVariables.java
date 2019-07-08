package ros2sy.code;

import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import ros2sy.sig.*;

public class InputVariables {
	private HashMap<String, Integer> typeCounts = new HashMap<String, Integer>();
	private HashMap<String, Type> plainTypeToType = new HashMap<String, Type>();
	private HashMap<Type, ArrayList<String>> namesByType = new HashMap<Type, ArrayList<String>>();
	private HashMap<String, Type> inputs;
	
	private ArrayList<ArrayList<Pair<String, Type>>> resultsAvailable = new ArrayList<ArrayList<Pair<String, Type>>>();
	
	public InputVariables() {
		this.inputs = new HashMap<String, Type>();
	}
	
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
//				namesByType.put(tipe, new ArrayList<String>());
			}
			if (!namesByType.containsKey(tipe)) {
				namesByType.put(tipe, new ArrayList<String>());
			}
			int old = typeCounts.get(t);
			typeCounts.put(t,  old + 1);
			namesByType.get(tipe).add(inputName);
		}
	}
	
	
	
	public void addInput(String name, String typeName) {
		Type t = new Type(typeName);
		if (hasTypeForPlain(t.getPlainType())) {
			String plain = t.getPlainType();
			t = typeForString(plain);
		}
		this.addInput(name,  t);
	}
	
	public void addInput(String name, Type t) {
		this.inputs.put(name, t);
		
		this.updateTypeCounts(t);
	}
	
	public HashMap<String, Type> getInputs() {
		return this.inputs;
	}
	
	public boolean contains(String typeName) {
		return hasTypeCountsForString(typeName) && hasTypeForPlain(typeName) && hasNamesForString(typeName);
	}
	
	public boolean hasTypeCountsForString(String typeName) {
		return this.typeCounts.containsKey(typeName);
	}
	
	private void updateTypeCounts(Type t) {
		this.updateTypeCounts(t.getPlainType());
	}
	
	private void updateTypeCounts(String typeString) {
		if (this.typeCounts.containsKey(typeString)) {
			int old = typeCounts.get(typeString);
			typeCounts.put(typeString, old + 1);
		} else {
			typeCounts.put(typeString, 1);
		}
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
	
	public void addNewResult(String name, Type t, int line) {
		if (resultsAvailable.size() <= line) {
			int size = resultsAvailable.size();
			for (int i = size; i < line + 1; i++) {
				resultsAvailable.add(new ArrayList<Pair<String, Type>>());
			}
		}
		
		resultsAvailable.get(line).add(new ImmutablePair<String, Type>(name, t));
	}
	
	public ArrayList<String> resultsOfTypeAvailable(int line, Type t) {
		if (line < 0) {
			line = 0;
		}
		if (line >= this.resultsAvailable.size()) {
			line = this.resultsAvailable.size() - 1;
		}
		
		ArrayList<String> names = new ArrayList<String>();
		if (line < this.resultsAvailable.size()) {			
			for (int i = line; i > -1; i--) {
				if (this.resultsAvailable.get(i).size() > 0) {
					for (int j = 0; j < this.resultsAvailable.get(i).size(); j++) {
						Pair<String, Type> pair = this.resultsAvailable.get(i).get(j);
						Type paramType = pair.getRight();
						if (paramType.asParamsEqual(t)) {
							names.add(pair.getLeft());
						} else if (paramType.isSharedPointer && (paramType.valueTypeName.equals(t.valueTypeName))) {
							names.add(pair.getLeft());
						} else {
							System.out.println("The type " + paramType.toString() + " and " + t.toString() + " are not equivalent as parameters.");
						}
					}
				}
			}
		}
		return names;
	}
}
