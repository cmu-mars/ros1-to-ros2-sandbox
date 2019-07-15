package ros2sy.sig;

import java.util.ArrayList;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * A class that represents a C++ type.
 * 
 * @author audrey
 */
public class Type {
	private static final Logger LOGGER = LogManager.getLogger(Type.class.getName());
	private String typeName;
	
	public boolean isReference;
	public int referenceLevel;
	
	private String refString = "";
	
	public boolean isPointer;
	public int pointerLevel;
	
	public boolean isSharedPointer = false;
	
	public boolean isConstValue = false;
	public ArrayList<Boolean> isConstPointer;
	
	public boolean isArrayType = false;
	public int arrayLevel = 0;
	
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
		
		if (name.indexOf("[]") > -1) {
			this.isArrayType = true;
			
			boolean lastCharWasLSqBracket = false;
			for (int i = 0; i < name.length(); i++) {
				if (name.charAt(i) == '[') {
					lastCharWasLSqBracket = true;
				} else if (lastCharWasLSqBracket) {
					if (name.charAt(i) == ']') {
						this.arrayLevel++;
					}
					lastCharWasLSqBracket = false;
				}
			}
		}
		
//		for (int i = 0; i < ints.size(); i++) {
//			LOGGER.info(name.substring(ints.get(i), name.length()));
//		}
		
		this.referenceLevel = ints.size();
		
		this.isReference = this.referenceLevel > 0;
		if (this.isReference) {
			this.typeName = this.typeName.substring(0, ints.get(0)).trim();
		}
		
		ints.clear();
		
		this.isConstPointer = new ArrayList<Boolean>();
		
		for (int i = 0; i < name.length(); i++) {
//			LOGGER.info(name.charAt(i));
//			LOGGER.info(name.charAt(i) == '*');
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
		
//		LOGGER.info(name);
//		LOGGER.info(this.isConstValue);
//		LOGGER.info(this.pointerLevel);
//		for (Boolean b : this.isConstPointer) {
//			LOGGER.info(b.booleanValue());
//		}
		
		this.valueTypeName = this.typeName.replaceAll("\\s*(const|\\*)\\s*", "");
		this.valueTypeName = this.valueTypeName.replaceAll("\\s?const\\s?", "");
		this.valueTypeName = this.valueTypeName.trim();
		
		LOGGER.trace("Value type name vs type name: {} -- {}", this.valueTypeName, this.typeName);
		
		if (valueTypeName.matches("^std::shared_ptr<.+>$")) {
//			LOGGER.info("This valueTypeName matches a shared pointer: <" + valueTypeName + ">, <" + this.typeName + ">");
			
			int first = "std::shared_ptr<".length();
			
			this.valueTypeName = this.valueTypeName.substring(first, (this.valueTypeName.indexOf('>') == this.valueTypeName.lastIndexOf('>') ) ? this.valueTypeName.lastIndexOf('>') : this.valueTypeName.length() );
			this.isSharedPointer = true;
		} else if (this.valueTypeName.indexOf("::SharedPtr") > -1) {
			int index = this.valueTypeName.indexOf("::SharedPtr");
			this.valueTypeName = this.valueTypeName.substring(0, index);
			this.isSharedPointer = true;
		}
		
		if (this.isArrayType) {
			this.valueTypeName = this.valueTypeName.replaceAll("\\[\\]", "");
			this.valueTypeName = this.valueTypeName.trim();
		}
		
		this.typeName = this.typeName.trim();
		
//		LOGGER.info("<" + this.valueTypeName + ">");
	}
	
	/**
	 * Gets the basic form of a type, i.e. the minimum required to be accepted
	 * as a parameter to a hole with this object's type.
	 * 
	 * For example, if we had Type tipe = new Type("const char*"), then the
	 * "plain" form of that type would be "char *". It essentially gets rid of
	 * "const" (as well as "&"), and formats the type's name in a standard way.
	 * 
	 * @return		a String representing the "minimalist" form of a type.
	 */
	public String getPlainName() {
		String plain = this.valueTypeName;
		
		if (this.isSharedPointer) {
			plain = "std::shared_ptr<" + plain + ">";
		}
		if (this.isPointer) {
			plain = plain + " ";
		}
		for (int i = 0; i < this.pointerLevel; i++) {
			plain = plain + "*";
		}
		
		if (this.isArrayType) {
			plain = plain + " ";
			for (int i = 0; i < this.arrayLevel; i++) {
				plain = plain + "[]";
			}
		}
		return plain;
	}
	
	public static String getPlainName(String typeName) {
		return (new Type(typeName)).getPlainName();
	}
	
	public Type getPlainType() {
		return new Type(this.getPlainName());
	}
	
	/**
	 * This tests whether this type is equivalent enough to fill the
	 * hole as an arg if other is the type of that hole.
	 * 
	 * @param other
	 * @return
	 */
	public boolean asParamsEqual(Type other) {
		return this.valueTypeName.equals(other.valueTypeName) && (this.isSharedPointer == other.isSharedPointer) && this.isArrayPointerEquivalent(other);
	}
	
	
	/**
	 * Figures out whether this type and another type are essentially the same
	 * pointer/array level, since C++ arrays are essentially just a bit of fancy
	 * pointer math.
	 * 
	 * @param other		another Type object to compare this instance to
	 * @return					whether these two types are essentially the same, pointer
	 * 								and array wise.
	 */
	public boolean isArrayPointerEquivalent(Type other) {
		return (this.isPointer == other.isPointer && this.isArrayType == other.isArrayType && this.pointerLevel == other.pointerLevel && this.arrayLevel == other.arrayLevel) || (this.pointerLevel + this.arrayLevel) == (other.pointerLevel + other.arrayLevel);
	}
	
	/**
	 * Returns the type of the string.
	 */
	@Override
	public String toString() {
		return this.typeName + refString;
	}
}
