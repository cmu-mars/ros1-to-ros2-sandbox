package ros2sy.code;

public class Type {
	private String typeName;
	
	public boolean isReference;
	
	
	public Type(String name) {
		this.typeName = name;
	}
	
	@Override
	public String toString() {
		return this.typeName;
	}
}
