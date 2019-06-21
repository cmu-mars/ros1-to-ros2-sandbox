package ros2sy.code;

import ros2sy.exception.ArgParseException;

/**
 * A class that represents an argument to a method, and holds the type
 * of the argument as well as the default value, in a method's signature.
 * 
 * @author audrey
 *
 */
public class Arg {
	public Type argType;
	public boolean hasDefault;
	public final String defaultValue;
	public final boolean takesMany;
	
	/**
	 * Constructor for the Arg class
	 * 
	 * @param myNewType	a Type object that is the type of this argument
	 */
	public Arg(Type myNewType) {
		this.argType = myNewType;
		this.hasDefault = false;
		this.defaultValue = "";
		this.takesMany = false;
	}
	
	/**
	 * Constructor for an Arg, that may have a default value.
	 * 
	 * If the default value is given as an empty string, it is assumed
	 * that this Arg has no default value. Therefore, if you want the
	 * default value to be an actual empty string, the string should
	 * really be "\"\"".
	 * 
	 * Similarly, all Strings should be surrounded in escaped quotes.
	 * 
	 * @param myNewType				the Type of this argument
	 * @param myDefaultValue		a String representing the default value of
	 * 												the arg
	 */
	public Arg(Type myNewType, String myDefaultValue) {
		this.argType = myNewType;
		this.hasDefault = myDefaultValue.length() > 0;
		this.defaultValue = myDefaultValue;
		this.takesMany = false;
	}
	
	/**
	 * Constructor for an Arg, given a String containing the type of the
	 * 
	 * 
	 * @param argString
	 * @throws ArgParseException
	 */
	public Arg(String argString) throws ArgParseException {
		this.takesMany = argString.indexOf("...") >= 0;
		
		if (this.takesMany) {
			argString = argString.substring(0, argString.indexOf("..."));
		}

		this.hasDefault = argString.indexOf("(=") > -1;
		if (this.hasDefault) {
			int lastIndex = argString.lastIndexOf(")");
			String[] splits = argString.substring(0, lastIndex).split("[ ]*\\(=[ ]*");
			
			if (splits.length == 2) {
				// There should really only be 2
				this.argType = new Type(splits[0]);
				this.defaultValue = splits[1];
			} else {
				this.defaultValue = "";
				throw new ArgParseException(
						"There found inappropriate number of default value tokens in " + argString + ", but there should be exactly one.");
			}
		} else {
			this.argType = new Type(argString.replaceAll("[ ]+", " "));
			this.defaultValue = "";
		}
	}
	
	/**
	 * 
	 */
	@Override
	public String toString() {
		return this.argType.toString() + ((this.hasDefault) ? " [" + this.defaultValue + "]" : "");
	}
	
}
