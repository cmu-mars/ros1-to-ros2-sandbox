package ros2sy.sig;

import java.util.ArrayList;

/**
 * A class that represents a C++ type.
 * 
 * @author audrey
 */

public class Type {
	private String typeName;
	
	public boolean isReference;
	public int referenceLevel;
	
	private String refString = "";
	
	public boolean isPointer;
	public int pointerLevel;
	
	public boolean isSharedPointer = false;
	
	public boolean isConstValue = false;
	public ArrayList<Boolean> isConstPointer;
	
	public String valueTypeName;
	
	/**
	 * Construct a Type object, given a string that contains the
	 * name of the type.
	 * 
	 * @param name		a String that contains the name of the type
	 */
	public Type(String name) {
		this.isConstValue = (name.matches("^\\s*const\\s*.*") || name.matches("^\\s*[^\\s]+\\s+const.*"));
		this.typeName = name;
		ArrayList<Integer> ints = new ArrayList<Integer>();
		
		for (int i = 0; i < name.length(); i++) {
			if (name.charAt(i) == '&') {
				
				ints.add(i);
				refString = refString + "&";
			}
		}
		
//		for (int i = 0; i < ints.size(); i++) {
//			System.out.println(name.substring(ints.get(i), name.length()));
//		}
		
		this.referenceLevel = ints.size();
		
		this.isReference = this.referenceLevel > 0;
		if (this.isReference) {
			this.typeName = this.typeName.substring(0, ints.get(0));
		}
		
		ints.clear();
		
		this.isConstPointer = new ArrayList<Boolean>();
		
		for (int i = 0; i < name.length(); i++) {
//			System.out.println(name.charAt(i));
//			System.out.println(name.charAt(i) == '*');
			if (name.charAt(i) == '*')	 {
				if (ints.size() > 0) {
					int lastInt = ints.get(ints.size() - 1);
					if (i - lastInt > 4) {
						this.isConstPointer.add(name.substring(lastInt + 1, i).matches("^\\s*const\\s*$"));
					} else {
						this.isConstPointer.add(false);
					}
				}
				
				ints.add(i);
			}
		}
		
		this.pointerLevel = ints.size();
		this.isPointer = this.pointerLevel > 0;
		
		if (this.pointerLevel > 0) {
			int last = ints.get(ints.size() - 1) + 1;
			String lastStr = name.substring(last, name.length());
			this.isConstPointer.add(lastStr.matches("^\\s*const\\s*.*"));
		}
		
//		System.out.println(name);
//		System.out.println(this.isConstValue);
//		System.out.println(this.pointerLevel);
//		for (Boolean b : this.isConstPointer) {
//			System.out.println(b.booleanValue());
//		}
		
		this.valueTypeName = this.typeName.replaceAll("\\s*(const|\\*)\\s*", "");
		this.valueTypeName = this.valueTypeName.trim();
		
		if (valueTypeName.matches("^std::shared_ptr<.+>$")) {
//			System.out.println("This valueTypeName matches a shared pointer: <" + valueTypeName + ">, <" + this.typeName + ">");
			
			int first = "std::shared_ptr<".length();
			
			this.valueTypeName = this.valueTypeName.substring(first, (this.valueTypeName.indexOf('>') == this.valueTypeName.lastIndexOf('>') ) ? this.valueTypeName.lastIndexOf('>') : this.valueTypeName.length() );
			this.isSharedPointer = true;
		} else if (this.valueTypeName.indexOf("::SharedPtr") > -1) {
			int index = this.valueTypeName.indexOf("::SharedPtr");
			this.valueTypeName = this.valueTypeName.substring(0, index);
			this.isSharedPointer = true;
		}
		
		this.typeName.trim();
		
//		System.out.println("<" + this.valueTypeName + ">");
	}
	
	public String getPlainType() {
		String plain = this.valueTypeName;
		if (this.isPointer) {
			plain = plain + " ";
		}
		for (int i = 0; i < this.pointerLevel; i++) {
			plain = plain + "*";
		}
		return plain;
	}
	
	/**
	 * This tests whether this type is equivalent enough to fill the
	 * hole as an arg if other is the type of that hole.
	 * 
	 * @param other
	 * @return
	 */
	public boolean asParamsEqual(Type other) {
		return this.valueTypeName.equals(other.valueTypeName) && (this.isSharedPointer == other.isSharedPointer) && (this.isPointer == other.isPointer) && (this.pointerLevel == other.pointerLevel);
	}
	
	/**
	 * Returns the type of the string.
	 */
	@Override
	public String toString() {
		return this.typeName + refString;
	}
}
