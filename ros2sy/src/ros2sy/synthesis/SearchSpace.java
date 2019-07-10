package ros2sy.synthesis;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import ros2sy.json.ParseJson;
import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.Method;

public class SearchSpace {
	private static final Logger LOGGER = LogManager.getLogger(SearchSpace.class.getName());
	HashMap<String, HashSet<String>> tagToNames;
	
	HashMap<String, HashSet<Method>> tagToMethods;
	
	ArrayList<ArrayList<ArrayList<String>>> searchBlocks = new ArrayList<>();
	
	
	public SearchSpace(ArrayList<Method> methods, String tagFileName, MethodsToPetriNet mtpn) {
		tagToMethods = ParseJson.tagMethods(methods, tagFileName);
		tagToNames = Method.methodSetsToStringSets(tagToMethods, mtpn);
	}
	
	private boolean hasAllTags(String [] tags) {
		for (int i = 0; i < tags.length; i++) {
			if (!this.tagToNames.containsKey(tags[i])) {
				return false;
			}
		}
		
		return true;
	}
	
	public ArrayList<String> getNameIntersect(String ...tags) {
		HashSet<String> nameIntersect = new HashSet<>();
		if (tags.length > 0 && this.hasAllTags(tags)) {
			nameIntersect.addAll(this.tagToNames.get(tags[0]));

			for (int i = 1; i < tags.length; i++) {
				nameIntersect.retainAll(this.tagToNames.get(tags[i]));
			}
		}
		return new ArrayList<String>(nameIntersect);
	}
	
	public void addBlock(String ...tags) {
		int index = searchBlocks.size();
		searchBlocks.add(new ArrayList<ArrayList<String>>());
		
		searchBlocks.get(index).add(this.getNameIntersect(tags));
	}
	
	public void addToLastBlock(String ...tags) {
		int index = searchBlocks.size() - 1;
		searchBlocks.get(index).add(this.getNameIntersect(tags));
	}
	
	public int numBlocks() {
		return this.searchBlocks.size();
	}
	
	public int numSetsPerBlock(int block) {
		return this.searchBlocks.get(block).size();
	}
	
	public ArrayList<String> othersExcept(int exception) {
		ArrayList<String> others = new ArrayList<>();
		
		for (int i = 0; i < this.numBlocks(); i++) {
			if (i != exception) {
				for (ArrayList<String> strs : this.searchBlocks.get(i)) {
					others.addAll(strs);
				}
			}
		}
		
		return others;
	}
	
	public ArrayList<String> othersExcept(int exception, ArrayList<String> dontUse) {
		ArrayList<String> others = this.othersExcept(exception);
		
		others.addAll(dontUse);
		return others;
	}
	
	public List<List<String>> getBlock(int index) {
		List<List<String>> strings = new ArrayList<>();
		
		for (int i = 0; i < this.numSetsPerBlock(index); i++) {
			strings.add(new ArrayList<String>(this.searchBlocks.get(index).get(i)));
		}
		return strings;
	}
}
