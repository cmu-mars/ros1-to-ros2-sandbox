package ros2sy.code;

import java.util.ArrayList;

public class Type {
	private String typeName;
	
	public boolean isReference;
	public int referenceLevel;
	
	private String refString = "";
	
	
	public Type(String name) {
		this.typeName = name;
		ArrayList<Integer> ints = new ArrayList<Integer>();
		for (int i = 0; i < name.length(); i++) {
			if (name.charAt(i) == '&') {
				ints.add(i);
				refString = refString + "&";
			}
		}
		
		for (int i = 0; i < ints.size(); i++) {
			System.out.println(name.substring(ints.get(i), name.length()));
		}
		if (ints.size() > 0) {
			this.typeName = this.typeName.substring(0, ints.get(0));
			this.isReference = true;
		} else {
			this.isReference = false;
		}
	}
	
	@Override
	public String toString() {
		return this.typeName + refString;
	}
}
