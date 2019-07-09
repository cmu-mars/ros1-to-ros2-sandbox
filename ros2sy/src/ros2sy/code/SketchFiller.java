package ros2sy.code;

import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

import ros2sy.exception.CodeGenerationException;
import ros2sy.json.ParseJson;
import ros2sy.petri.MethodsToPetriNet;
import ros2sy.sig.Method;

public class SketchFiller {
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
	
	public void fillSketches(String sketchPath, InputVariables ivs) throws CodeGenerationException {
		String sketchBaseName = getSketchBaseName(sketchPath);
		
		ArrayList<String> code = this.filler.generateCodeWithInputs(ivs.getInputs());
		// Get the sketch
		try {
			String sketchContents = new String(Files.readAllBytes(Paths.get(sketchPath)));
			ArrayList<String> filledOutSketches = new ArrayList<String>();
			for (String fill : code) {
				String [] splits = fill.split("\n");
				String replaceTag = "\\?#\\?";
				String sketch = sketchContents;
				int i = 0;
				while (sketch.indexOf("?#?") > -1 && i < splits.length) {
					System.out.println("Replacing an instance of ?#? in the replace string.");
					sketch = sketch.replaceFirst(replaceTag, splits[i]);
					i++;
				}
				filledOutSketches.add(sketch);
			}
			
			this.replaceIncludes(filledOutSketches);
			this.replaceTypes(filledOutSketches);
			
			

			int times = 0;
			for (String sketch : filledOutSketches) {
				System.out.println("the full sketch: ");
				System.out.println(sketch);

				try {
					FileWriter fw = new FileWriter(sketchBaseName + Integer.toString(times) + ".cpp");

					fw.write(sketch);
					fw.flush();
					fw.close();
				} catch (Exception e) {
					System.out.println(e);
				}
				times++;
			}
		} catch (Exception e) {
			System.out.println("Caught an exception: ");
			System.out.println(e);
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
