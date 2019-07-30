package ros2sy.code;

import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import ros2sy.exception.CodeGenerationException;
import ros2sy.json.ParseJson;
import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.Method;

public class SketchFiller {
	private static final Logger LOGGER = LogManager.getLogger(SketchFiller.class.getName());
	CppCode filler;
	MethodsToPetriNet mpn;
	
	public SketchFiller(MethodsToPetriNet mpn, ArrayList<String> codes) {
		this.filler = new CppCode(mpn, codes);
		this.mpn = mpn;
	}
	
	public void replaceIncludes(ArrayList<String> codes) {
		HashMap<String, String> includes = ParseJson.includesToReplace();
		
		this.replaceInAll(codes, includes);
		
		for (int i = 0; i < codes.size(); i++) {
			String c = codes.get(i);
			
			for (String incl : filler.getIncludes()) {
				String inclusion = "#include \"" + incl + "\"";
				if (c.indexOf(inclusion) == -1) {
					c = inclusion + "\n" + c;					
				}
			}
			
			codes.set(i,  c);
		}
	}
	
	public void replaceInAll(ArrayList<String> codes, HashMap<String, String> replacers) {
		for (String i : replacers.keySet()) {
			for (int j = 0; j < codes.size(); j++) {
				if (codes.get(j).indexOf(i) > -1) {
					codes.set(j, codes.get(j).replace(i, replacers.get(i)));
				}
			}
		}
	}
	
	public void replaceTypes(ArrayList<String> codes) {
		HashMap<String, String> types = ParseJson.getDictionary("ros1-to-ros2-code-spec.json", "types");
		this.replaceInAll(codes,  types);
	}
	
	public ArrayList<String> getTypesAvailable(String code) {
		ArrayList<String> includesGiven = new ArrayList<>();
		
		String [] lines = code.split("\n");
		
		for (String line : lines) {
			if (line.indexOf("#include") > -1) {
				String inclusion = "";
				if (line.matches("#include <([^>]*)>")) {
					inclusion = line.replaceAll("#include <([^>]*)>", "$1");
				} else if (line.matches("#include \"([^\"]*)\"")) {
					inclusion = line.replaceAll("#include \"([^>]*)\"", "$1");
				}
				
				LOGGER.debug("Inclusion found: <{}>", inclusion);
				
				includesGiven.add(inclusion);
			}
		}
		
		return includesGiven;
	}
	
	
	private ArrayList<HoleType> getHoleTypes(String sketch) {
		int index = 0;
		ArrayList<HoleType> holes = new ArrayList<>();
//		String statementHole = "\\?#\\?";
//		String exprHole = "\\?\\(#\\)\\?";
		
		int lastIndex = 0;
		while (index > -1 && index < sketch.length()) {
			lastIndex = index;
			
			index = sketch.indexOf("?#?", lastIndex);
			int tempIndex = sketch.indexOf("?(#)?", lastIndex);
			if ((index == -1 || index > tempIndex) && tempIndex > -1) {
				index = tempIndex;
				holes.add(HoleType.EXPRESSION);
			} else {
				holes.add(HoleType.STATEMENT);
			}
		}
		
		return holes;
	}
	
	public void fillSketches(String sketchPath, InputVariables ivs) throws CodeGenerationException {
		String sketchBaseName = getSketchBaseName(sketchPath);
		
		// Get the sketch
		try {
			String sketchContents = new String(Files.readAllBytes(Paths.get(sketchPath)));
			
			getTypesAvailable(sketchContents);
			
			ArrayList<String> code = this.filler.generateCodeWithInputs(ivs.getInputs());
			
			ArrayList<String> filledOutSketches = new ArrayList<String>();
			for (String fill : code) {
				String [] splits = fill.split("\n");
				String replaceTag = "\\?#\\?";
				String sketch = sketchContents;
				int i = 0;
				while (sketch.indexOf("?#?") > -1 && i < splits.length) {
					LOGGER.trace("Replacing an instance of ?#? in the replace string with <{}>.", splits[i]);
					sketch = sketch.replaceFirst(replaceTag, splits[i]);
					i++;
				}
				filledOutSketches.add(sketch);
			}
			
			this.replaceIncludes(filledOutSketches);
			this.replaceTypes(filledOutSketches);
			
			

			int times = 0;
			for (String sketch : filledOutSketches) {
				LOGGER.trace("the full sketch:", sketch);

				try {
					FileWriter fw = new FileWriter(sketchBaseName + Integer.toString(times) + ".cpp");

					fw.write(sketch);
					fw.flush();
					fw.close();
				} catch (Exception e) {
					LOGGER.info(e);
				}
				times++;
			}
		} catch (Exception e) {
			LOGGER.warn("Caught an exception: ", e);
		}
	}
	
	private String getSketchBaseName(String fileName) {
		int indexOfExtension = fileName.lastIndexOf(".sketch");
		if (indexOfExtension == -1) {
			indexOfExtension = fileName.length();
		}
		return fileName.substring(0, indexOfExtension);
	}
}
