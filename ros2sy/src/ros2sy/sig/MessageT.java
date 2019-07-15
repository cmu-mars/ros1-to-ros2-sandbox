package ros2sy.sig;

public class MessageT extends TemplateParameter {
	
	public MessageT(String name) {
		super(name);
	}
	
	public MessageT(String name, String defValue) {
		super(name, defValue);
	}

	@Override
	public boolean fitsParameter(Type t) {
		return t.toString().indexOf("::msgs::") > -1 || t.toString().equals(this.name);
	}
}