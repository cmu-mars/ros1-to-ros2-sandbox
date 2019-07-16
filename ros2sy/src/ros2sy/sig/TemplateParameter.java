package ros2sy.sig;

public class TemplateParameter {
	public String name;
	public boolean hasDefault;
	private String defaultValue;
	
	public TemplateParameter(String name) {
		this.name = name;
		this.hasDefault = false;
		
		this.defaultValue = "";
	}
	
	public TemplateParameter(String name, String defValue) {
		this.name = name;
		
		this.hasDefault = (defValue.length() > 0);
		if (this.hasDefault) {
			this.defaultValue = defValue;
		}
	}
	
	public boolean fitsParameter(Type t) {
		return t.toString().equals(this.name);
	}
	
	public String replaceWithName(String base, String replacement) {
		return base.replaceAll(this.name, replacement);
	}
}