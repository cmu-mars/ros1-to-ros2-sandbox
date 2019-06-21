package ros2sy.petri;
import ros2sy.code.*;
import ros2sy.exception.ArgParseException;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import uniol.apt.adt.exception.FlowExistsException;
import uniol.apt.adt.exception.NoSuchEdgeException;
import uniol.apt.adt.pn.*;

/**
 * The MethodsToPetriNet contains a handful of helpful
 * methods for creating a petri net from your methods,
 * and then also for outputting a dot file displaying
 * the petri net.
 * 
 * @author audrey
 *
 */
public class MethodsToPetriNet {
	
	/**
	 * 
	 * @param methods
	 * @return
	 */
	public static PetriNet convert(ArrayList<Method> methods) {
		PetriNet pn = new PetriNet();
		
		HashMap<String, Integer> nameCounts = new HashMap<String, Integer>();
		
		for (Method m : methods) {
			if (!nameCounts.containsKey(m.name)) {
				nameCounts.put(m.name,	0);
			}
			nameCounts.put(m.name, nameCounts.get(m.name) + 1);
			
			Transition t = MethodsToPetriNet.makeMethodTransition(m, pn, nameCounts);				
			if (m.isClassMethod) {
				String instanceType = m.fromClass.toString();
				
				Place inst = MethodsToPetriNet.getPetriPlace(pn, instanceType);
				pn.createFlow(inst, t);
			}
			
			
			String returnType = m.returnType.toString();
			Place ret = MethodsToPetriNet.getPetriPlace(pn, returnType);
			if (m.returnType.isSharedPointer) {
				String internalValueType = m.returnType.valueTypeName;
				String transName = internalValueType + "::Shared_to_unshared";
				MethodsToPetriNet.addTransition(pn, returnType, transName, internalValueType);
				
//				Place internVal = MethodsToPetriNet.getPetriPlace(pn, internalValueType);
//				
//				Transition unptr = (pn.containsTransition(transName)) ? pn.getTransition(transName) : pn.createTransition(transName);
//				
//				
//				MethodsToPetriNet.addFlowToPetri(pn, ret, unptr);
//				MethodsToPetriNet.addFlowToPetri(pn, unptr, internVal);
			}
			
			if (!m.hasOptionalArgs()) {
				MethodsToPetriNet.addTransitionFlowToPetri(pn, m.args, t, ret);
			} else {
				
				ArrayList<Arg> mandatory = m.getMandatoryArgs();
				if (m.isClassMethod) {
					String instType = m.fromClass.toString();
					try {
						mandatory.add(new Arg(instType));
					} catch (ArgParseException e) {
						System.out.println("ArgParseException occurred in MethodsToPetriNet.convert");
						System.out.println(e);
					}
				}
				
				MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, t, ret);
				
				ArrayList<Arg> options = m.getOptionalArgs();
				
				
				for (int i = 0; i < options.size(); i++) {
					Transition opt = pn.createTransition(t.getId() + "(ALT-" + Integer.toString(i) + ")");
					mandatory.add(options.get(i));
					
					
					
					MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, opt, ret);
				}	
			}
		}
		
		Set<Place> places = pn.getPlaces();
		Place[] placeArray = new Place [places.size()];
		places.toArray(placeArray);
		
		
		for (int i = 0; i < placeArray.length-1; i++) {
			for (int j = i+1; j < placeArray.length; j++) {
				Place a = placeArray[i];
				Place b = placeArray[j];
				String aid = a.getId();
				String bid = b.getId();
				Type ta = new Type(MethodsToPetriNet.removeOptSuffix(aid));
				Type tb = new Type(MethodsToPetriNet.removeOptSuffix(bid));
				
				if (ta.asParamsEqual(tb)) {
					boolean decider = (ta.isConstValue || ta.isReference) || MethodsToPetriNet.isOptional(pn.getNode(aid));
					
					String source = (decider) ? bid : aid;
					String target = (decider) ? aid : bid;
//					String a_or_b = source + "_to_" + target;
					
					if (!source.endsWith("(OPT)")) {
						MethodsToPetriNet.addTransition(pn, source, target);						
					}
					
				} else if (ta.valueTypeName.contentEquals("rclcpp::Node") && tb.valueTypeName.contentEquals("rcl_node_t") && (ta.isSharedPointer == tb.isSharedPointer)) {
					MethodsToPetriNet.addTransition(pn, aid, bid);
				}
			}
		}
		
		for (int i = 0; i < placeArray.length; i++) {
			if (!MethodsToPetriNet.isOptional(placeArray[i])) {
				MethodsToPetriNet.addClone(pn, placeArray[i]);
				
			}
		}
		
		if (pn.containsPlace("size_t") && pn.containsPlace("int")) {
			Place size_t = pn.getPlace("size_t");
			Place integer = pn.getPlace("int");
			
			Transition convert = pn.createTransition("int_to_size_t");
			
			MethodsToPetriNet.addFlowToPetri(pn, integer, convert);
			MethodsToPetriNet.addFlowToPetri(pn, convert, size_t);
		}
		
		if (pn.containsPlace("char const *const") && !pn.containsPlace("char**")) {
			MethodsToPetriNet.addTransition(pn, "char**", "char const *const");
			MethodsToPetriNet.addClone(pn, pn.getPlace("char**"));
		}
		
		if (pn.containsPlace("rclcpp::Node::SharedPtr") && pn.containsPlace("std::shared_ptr<rclcpp::Node>")) {
			MethodsToPetriNet.addTransition(pn, "rclcpp::Node::SharedPtr", "std::shared_ptr<rclcpp::Node>");
			
			MethodsToPetriNet.addTransition(pn, "std::shared_ptr<rclcpp::Node>", "rclcpp::Node::SharedPtr");
			
		}
		return pn;
	}
	
	private static void addClone(PetriNet pn, Place p) {
		Transition t = pn.createTransition(p.getId() + "_clone");
		pn.createFlow(p, t);
		Flow f = pn.createFlow(t, p);
		f.setWeight(2);
		
	}
	
	private static String removeOptSuffix(String name) {
		int index = name.indexOf("(OPT)");
		if (index < 0) {
			return name;
		}
		return name.substring(0, index);
	}
	
	
	/**
	 * 
	 * @param pn
	 * @param id
	 * @return
	 */
	private static Place getPetriPlace(PetriNet pn, String id) {
		return (!pn.containsPlace(id)) ? pn.createPlace(id) : pn.getPlace(id);
	}
	
	/**
	 * 
	 * @param pn
	 */
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
				w.write("\t\"" + t.getId() + "\" [shape=box,style=filled,color=black,fontcolor=white];\n");
			}
			
			HashMap<Node, Set<Node>> nodeMap = new HashMap<Node, Set<Node>>();
			boolean dashed = false;
			
			Set<Flow> flows = pn.getEdges();
			
			for (Flow f : flows) {
				Node n = f.getSource();
				Node p = f.getTarget();
				
				w.write("\"" + n.getId() + "\" -> \"" + p.getId() + "\" [label=\"" + Integer.toString(f.getWeight()) + "\"");
				
				if (MethodsToPetriNet.isOptional(n)) {
					w.write(",style=dotted");
				}
				
				if (n.getId().equals("const std::string&")) {
					w.write(",color=red");
				} else if (n.getId().equals("CallbackT &&")) {
					w.write(",color=blue");
				} else if (n.getId().equals("bool(OPT)")) {
					w.write(",color=orange");
				}
				
				w.write("];\n");
			}
			
			
//			for (Node n : nodes) {
//				nodeMap.put(n, new HashSet<Node>());
//				
//				HashSet<Node> posts = new HashSet<Node>(pn.getPostsetNodes(n));
//				if (posts.size() > 0) {
//					w.write("\t\"" + n.getId() + "\" -> {");
//					for (Node p : posts) {
//						
//						w.write("\"" + p.getId() + "\" ");	
//					}
//					w.write("}");
//					
//					if (MethodsToPetriNet.isOptional(n)) {
//						// Just style the optional edges as
//						// either dotted or dashed, so that
//						// they're visually lightened, and
//						// there's at least some variety so
//						// it's easier to discern which edges
//						// came from where
//						String style = (dashed) ? "dotted" : "dashed";
//						w.write(" [style=" + style + "]");
//						dashed = !dashed;
//					}
//					if (n.getId().equals("const std::string&")) {
//						w.write(" [color=red]");
//					} else if (n.getId().equals("CallbackT &&")) {
//						w.write(" [color=blue]");
//					} else if (n.getId().equals("bool(OPT)")) {
//						w.write(" [color=orange]");
//					}
//					w.write(";\n");
//				}
//			}
//			w.write("}\n");
			w.write("}\n");
			
			w.flush();
			w.close();
		} catch (Exception e) {
			System.out.println("Exception caught while attempting to write dot file in MethodsToPetriNet.");
			System.out.println(e);
		}
	}
	
	/**
	 * Adds flows that go from arguments to the function that requires
	 * them.
	 * 
	 * Also marks optional arguments with "(OPT)" afterwards.
	 * 
	 * @param pn			the PetriNet to add the flows to
	 * @param t			the Transition that represents the function
	 * @param args		the Arg objects representing args to that function
	 */
	private static void addArgFlowsToPetri(PetriNet pn, Transition t, ArrayList<Arg> args) {
		for (Arg a : args) {
			String strType = a.argType.toString();
			if (a.hasDefault) {
				strType = strType + "(OPT)";
			}
			Place p = (!pn.containsPlace(strType)) ? pn.createPlace(strType) : pn.getPlace(strType);
			MethodsToPetriNet.addFlowToPetri(pn, p, t);
			
//			try {
//				// Creates a flow with default weight 1
//				pn.createFlow(p, t);
//			} catch (FlowExistsException e) {
//				System.out.println("Caught exception in MethodsToPetriNet.addArgFlowsToPetri:");
//				System.out.println(e);
//				
//				try {
//					Flow f = pn.getFlow(p.getId(), t.getId());
//					int wt = f.getWeight();
//					
//					// Increase the weight
//					f.setWeight(wt + 1);
//				} catch (NoSuchEdgeException e1) {
//					System.out.println("Caught exception in MethodsToPetriNet.addArgFlowsToPetri:");
//					System.out.println(e1);
//				}
//			}
		}
	}
	
	private static void addTransition(PetriNet pn, String p1_id, String t_id, String p2_id) {
		Place p1 = MethodsToPetriNet.getPetriPlace(pn, p1_id);
		Place p2 = MethodsToPetriNet.getPetriPlace(pn, p2_id);
		if (!pn.containsTransition(t_id))	 {
			Transition t = pn.createTransition(t_id);
			pn.createFlow(p1, t);
			pn.createFlow(t, p2);
		}
	}
	
	private static void addTransition(PetriNet pn, String p1_id, String p2_id) {
		String t_name = p1_id + "_to_" + p2_id;
		if (!pn.containsTransition(t_name)) {
			MethodsToPetriNet.addTransition(pn, p1_id, t_name, p2_id);
		}
		
	}
	
	private static void addFlowToPetri(PetriNet pn, Node p, Node t) {
		try {
			// Creates a flow with default weight 1
			pn.createFlow(p, t);
		} catch (FlowExistsException e) {
			System.out.println("Caught exception in MethodsToPetriNet.addFlowToPetri:");
			System.out.println(e);
			
			try {
				Flow f = pn.getFlow(p.getId(), t.getId());
				int wt = f.getWeight();
				
				// Increase the weight
				f.setWeight(wt + 1);
			} catch (NoSuchEdgeException e1) {
				System.out.println("Caught exception in MethodsToPetriNet.addFlowToPetri:");
				System.out.println(e1);
			}
		}
	}
	
	/**
	 * Creates a brand new transition based on a given method in the 
	 * given PetriNet, with a fresh name, even if the method is 
	 * overloaded.
	 * 
	 * @param m				the Method to be added
	 * @param pn				the PetriNet to add a transition representing m
	 * 								to
	 * @param counts		a HashMap<String, Integer> that records the
	 * 								the number of times each method's name has been
	 * 								seen, so that overloaded methods can be put in
	 * 								as distinct transitions
	 * @return					a Transition with a name that doesn't collide
	 * 								with other Transitions in the PetriNet pn
	 */
	private static Transition makeMethodTransition(Method m, PetriNet pn, HashMap<String, Integer> counts) {
		return (!pn.containsTransition(m.name)) ? pn.createTransition(m.name) : pn.createTransition(m.name + counts.get(m.name).toString());
	}
	
	/**
	 * Adds both flows for a method transition -- the incoming edges
	 * from args and the outgoing edges to the return type -- to the
	 * given PetriNet.
	 * 
	 * @param pn			the PetriNet to add the flows to
	 * @param args		an ArrayList<Arg> containing the arguments to add
	 * 							flows from
	 * @param t			the Transition representing the method of interest
	 * @param post		a Place representing the return type of the method
	 */
	private static void addTransitionFlowToPetri(PetriNet pn, ArrayList<Arg> args, Transition t, Place post) {
		MethodsToPetriNet.addArgFlowsToPetri(pn, t, args);
		
		// This automatically has a default weight of 1;
		pn.createFlow(t, post);
	}
	

	/**
	 * Determines whether the given node represents an optional arg,
	 * given that the node was created by using the method
	 * MethodsToPetriNet.convert.
	 * 
	 * @param n			the Node in question
	 * @return				boolean, whether or not this Node is actually
	 * 							optional
	 */
	private static boolean isOptional(Node n) {
		return n.getId().indexOf("OPT") > -1;
	}
	
	/**
	 *	 Finds the set of all nodes in a given HashSet of Node objects
	 * that are optional.
	 * 
	 * @param nodes 		the HashSet<Node> object containing the nodes
	 * 								that we want to find the optional subset from
	 * @return					a HashSet<Node> containing the subset of 
	 * 								optional nodes
	 */
	private static HashSet<Node> optionalNodes(HashSet<Node> nodes) {
		HashSet<Node> optionals = new HashSet<Node>();
		for (Node opt : nodes) {
			if (MethodsToPetriNet.isOptional(opt)) {
				optionals.add(opt);
			}
		}
		return optionals;
	}
	

}
