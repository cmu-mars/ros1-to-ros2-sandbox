package ros2sy.exception;

public class CodeGenerationException extends Exception {
	private String description;
	static final long serialVersionUID=11;
	
	public CodeGenerationException(String description) {
		this.description = description;
	}
	
	public String toString() {
		return "[CodeGenerationException]: \"" + this.description + "\"";
	}
}
