package ros2sy.sig;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import ros2sy.json.ParseJson;

import java.util.HashMap;
import java.util.Set;

public class TemplateParameter {
	private static Logger LOGGER = LogManager.getLogger(TemplateParameter.class.getName());
	
	public String name;
	public boolean hasDefault;
	private String defaultValue;
	private boolean isClassTemplate = false;
	
	private static HashMap<String, String> parameterTypeRegexes = ParseJson.getParameterMatchMap("inputs/param-map.json");
	
	public TemplateParameter(String name) {
		this.name = name;
		this.hasDefault = false;
		
		this.defaultValue = "";
	}
	
	public TemplateParameter(String name, boolean isClassTemplate) {
		this.name = name;
		this.isClassTemplate = isClassTemplate;
		this.hasDefault = false;
	}
	
	public TemplateParameter(String name, String defValue) {
		this.name = name;
		
		this.hasDefault = (defValue.length() > 0);
		if (this.hasDefault) {
			this.defaultValue = defValue;
		}
	}
	
	public TemplateParameter(String name, String defValue, boolean isClassTemplate) {
		this.name = name;
		this.hasDefault = (defValue.length() > 0);
		if (this.hasDefault) {
			this.defaultValue = defValue;
		}
		this.isClassTemplate = isClassTemplate;
	}
	
	public boolean fitsParameter(Type t) {
		return t.toString().equals(this.name);
	}
	
	public String replaceWithName(String base, String replacement) {
		return base.replaceAll(this.name, replacement);
	}
	
	public static boolean isMessageTEquivalent(String s) {
		return s.indexOf("::msgs::") > -1;
	}
	
	public static boolean hasParameterKey(String key) {
		return parameterTypeRegexes.containsKey(key);
	}
	
	public static Set<String> parameterKeys() {
		return parameterTypeRegexes.keySet();
	}
	
	public static boolean isParameterEquivalent(String key, String typeName) {
		if (!hasParameterKey(key)) {
			LOGGER.trace("You attempted to use key <{}>, which cannot be found in our dictionary of regexes for identifying parametric types: {}", key, parameterTypeRegexes);
			return false;
		}
		return typeName.matches(parameterTypeRegexes.get(key));
	}
	
	public String getDefaultValue() {
		return this.defaultValue;
	}
	
	public boolean isClassTemplate() {
		return this.isClassTemplate;
	}
	
	@Override
	public String toString() {
		return this.name + ((this.hasDefault) ? " = " + this.defaultValue : "");
	}
}
