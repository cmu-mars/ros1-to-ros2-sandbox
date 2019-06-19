package ros2sy.code;

public class ArgParseException extends Exception {
	private String description;
	static final long serialVersionUID = 10;
	public ArgParseException(String description) {
		this.description = description;
	}
	
	public String toString() {
		return "[ArgParseException]: \"" +  this.description + "\"";
	}
}
