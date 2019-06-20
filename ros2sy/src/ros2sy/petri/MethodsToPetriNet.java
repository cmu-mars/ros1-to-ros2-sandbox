package ros2sy.petri;
import ros2sy.code.*;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import uniol.apt.adt.pn.*;

public class MethodsToPetriNet {
	
	public static void addArgFlowsToPetri(PetriNet pn, Transition t, ArrayList<Arg> args) {
		for (Arg a : args) {
			String strType = a.argType.toString();
			if (a.hasDefault) {
				strType = strType + "(OPT)";
			}
			Place p = (!pn.containsPlace(strType)) ? pn.createPlace(strType) : pn.getPlace(strType);
			try {
				pn.createFlow(p, t);
			} catch (Exception e) {
				System.out.println("Caught exception in converting methods to petri net: ");
				System.out.println(e);
			}
		}
	}
	
	public static Transition makeMethodTransition(Method m, PetriNet pn, HashMap<String, Integer> counts) {
		return (!pn.containsTransition(m.name)) ? pn.createTransition(m.name) : pn.createTransition(m.name + counts.get(m.name).toString());
	}
	
	public static void addTransitionFlowToPetri(PetriNet pn, ArrayList<Arg> args, Transition t, Place post) {
		MethodsToPetriNet.addArgFlowsToPetri(pn, t, args);
		
		pn.createFlow(t, post);
	}
	
	public static PetriNet convert(ArrayList<Method> methods) {
		PetriNet pn = new PetriNet();
		
		HashMap<String, Integer> nameCounts = new HashMap<String, Integer>();
		
		for (Method m : methods) {
			if (!nameCounts.containsKey(m.name)) {
				nameCounts.put(m.name,	0);
			}
			nameCounts.put(m.name, nameCounts.get(m.name) + 1);
			
			
			String returnType = m.returnType.toString();
			Place ret = (!pn.containsPlace(returnType)) ? pn.createPlace(returnType) : pn.getPlace(returnType);
			if (!m.hasOptionalArgs()) {
				Transition t = MethodsToPetriNet.makeMethodTransition(m, pn, nameCounts);				
				
				MethodsToPetriNet.addTransitionFlowToPetri(pn, m.args, t, ret);
			} else {
				Transition man = MethodsToPetriNet.makeMethodTransition(m, pn, nameCounts);
				
				ArrayList<Arg> mandatory = m.getMandatoryArgs();
				MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, man, ret);
				
				ArrayList<Arg> options = m.getOptionalArgs();
				for (int i = 0; i < options.size(); i++) {
					Transition opt = pn.createTransition(man.getId() + "(ALT-" + Integer.toString(i) + ")");
					mandatory.add(options.get(i));
					
					MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, opt, ret);
				}
				
			}
			
			
		}
		
		return pn;
	}
	
	public static boolean isOptional(Node n) {
		return n.getId().indexOf("OPT") > -1;
	}
	
	public static HashSet<Node> optionalNodes(HashSet<Node> nodes) {
		HashSet<Node> optionals = (HashSet<Node>) nodes.clone();
		for (Node opt : optionals) {
			if (!MethodsToPetriNet.isOptional(opt)) {
				optionals.remove(opt);
			}
		}
		return optionals;
	}
	
	public static void createDotFile(PetriNet pn) {
		try {			
			FileWriter w = new FileWriter("dot/petri.dot");
			
			w.write("digraph {\n");
			w.write("\trankdir=LR;");
			w.write(" nodesep=1;");
			w.write("\tranksep=2;\n");
			
			Set<Node> nodes = pn.getNodes();
			
			Set<Transition> trans = pn.getTransitions();
			
			for (Transition t : trans) {
				w.write("\t\"" + t.getId() + "\" [shape=box];");
			}
			
			HashMap<Node, Set<Node>> nodeMap = new HashMap<Node, Set<Node>>();
			
			for (Node n : nodes) {
				nodeMap.put(n, new HashSet<Node>());
				
				HashSet<Node> posts = new HashSet<Node>(pn.getPostsetNodes(n));
				if (posts.size() > 0) {
					w.write("\t\"" + n.getId() + "\" -> {");
					for (Node p : posts) {
						
						w.write("\"" + p.getId() + "\" ");	
					}
					w.write("}");
					
					if (MethodsToPetriNet.isOptional(n)) {
						w.write(" [style=dotted]");
					}
					if (n.getId().equals("const std::string&")) {
						w.write(" [color=red]");
					}
					w.write(";\n");
				}
			}
			w.write("}\n");
			w.flush();
			w.close();
		} catch (Exception e) {
			System.out.println("Exception caught while attempting to write dot file in MethodsToPetriNet.");
			System.out.println(e);
		}
		
		
	}
}
