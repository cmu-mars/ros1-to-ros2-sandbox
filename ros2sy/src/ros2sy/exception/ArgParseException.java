package ros2sy.exception;

/**
 * 
 * @author audrey
 *
 */
public class ArgParseException extends Exception {
	private String description;
	static final long serialVersionUID = 10;
	
	/**
	 * 
	 * @param description
	 */
	public ArgParseException(String description) {
		this.description = description;
	}
	
	/**
	 * 
	 */
	public String toString() {
		return "[ArgParseException]: \"" +  this.description + "\"";
	}
}
