package ros2sy.code;

public class Arg {
	public Type argType;
	public boolean hasDefault;
	public final String defaultValue;
	public final boolean takesMany;
	
	public Arg(Type myNewType) {
		this.argType = myNewType;
		this.hasDefault = false;
		this.defaultValue = "";
		this.takesMany = false;
	}
	
	public Arg(Type myNewType, String myDefaultValue) {
		this.argType = myNewType;
		this.hasDefault = myDefaultValue.length() > 0;
		this.defaultValue = myDefaultValue;
		this.takesMany = false;
	}
	
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
	
	@Override
	public String toString() {
		return this.argType.toString() + ((this.hasDefault) ? " [" + this.defaultValue + "]" : "");
	}
	
}
