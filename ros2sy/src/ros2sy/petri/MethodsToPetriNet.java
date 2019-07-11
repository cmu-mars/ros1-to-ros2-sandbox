package ros2sy.petri;
import ros2sy.exception.ArgParseException;
import ros2sy.sig.*;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

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
	private static final Logger LOGGER = LogManager.getLogger(MethodsToPetriNet.class.getName());
	public static String un_pointer_name = "*DE_PTR*";
	
	/**
	 * The fake Method signature for dummy transitions such as clones and fake
	 * "type casts".
	 * This provides an easy way of identifying whether the method a transition
	 * represents is actually a real API.
	 */
	private Method dummy = new Method("*DUMMY*", new ArrayList<String>(), "void");
	
	/**
	 * The fake method that transitions use for taking a
	 * SharedPtr/std::shared_ptr to the object inside. This is really just a way
	 * of encoding the pointer field accesses.
	 */
	private Method de_ptr = new Method(un_pointer_name, new ArrayList<String>(), "void");
	ArrayList<String> silly;
	HashMap<String, String> typeAliases;
	ArrayList<Method> methods;
	public PetriNet net;
	HashMap<String, Method> methodAliases;
	
	HashMap<Method, ArrayList<String>> aliasesForMethods = new HashMap<Method, ArrayList<String>>();
	
	/**
	 * Instantiate the a MethodsToPetriNet object, which contains information
	 * about how the APIs of interest relate to a PetriNet.
	 * 
	 * @param methods			the ArrayList of Method objects
	 * @param dontClone		an ArrayList, containing descriptions of types that
	 * 										should not have clone edges.
	 */
	public MethodsToPetriNet(ArrayList<Method> methods, ArrayList<String> dontClone) {
		this.methods = methods;
		this.silly = dontClone;
		this.typeAliases = new HashMap<String, String>();
		
		this.typeAliases.put("int", "size_t");
		this.methodAliases = new HashMap<String, Method>();
		
		// Get a bare-bones petri net, containing only what you can gather from
		// the methods themselves.
		this.net = MethodsToPetriNet.convert(this, this.methods);
		
		Set<Place> places = this.net.getPlaces();
		Place[] placeArray = new Place [places.size()];
		places.toArray(placeArray);
		
		this.addParamEquivalences();
		
		// Add clones
		for (int i = 0; i < placeArray.length; i++) {
			if (!this.isOptional(placeArray[i]) && !this.silly.contains(placeArray[i].getId())) {
				MethodsToPetriNet.addClone(net, placeArray[i]);
			}
		}
		
		if (net.containsPlace("size_t") && net.containsPlace("int")) {
			Place size_t = net.getPlace("size_t");
			Place integer = net.getPlace("int");
			
			MethodsToPetriNet.addTransition(net, "int", "size_t");
		}
		
		if (net.containsPlace("char const *const") && !net.containsPlace("char**")) {
			MethodsToPetriNet.addTransition(net, "char**", "char const *const");
			this.addClone(net.getPlace("char**"));
		}
		
		if (net.containsPlace("rclcpp::Node::SharedPtr") && net.containsPlace("std::shared_ptr<rclcpp::Node>")) {
			MethodsToPetriNet.addTransition(net, "rclcpp::Node::SharedPtr", "std::shared_ptr<rclcpp::Node>");
			
			MethodsToPetriNet.addTransition(net, "std::shared_ptr<rclcpp::Node>", "rclcpp::Node::SharedPtr");
		}
		
		for (Place p : places) {
			int max = 0;
			for (Flow f : p.getPostsetEdges()) {
				if (f.getWeight() > max) {
					max = f.getWeight();
				}
			}
			p.setMaxToken(max + 1);
		}
	}
	
	
	/**
	 * Useful for determining whether a given Method object is actually just
	 * a placeholder for pointer field access notation.
	 * 
	 * @param m		a method object
	 * @return			a boolean that holds whether m is equal to the placeholder
	 * 						method for pointer field accesses
	 */
	public boolean isPointerFieldAccess(Method m) {
		return m.equals(de_ptr);
	}
	
	public boolean isNotDummyMethod(Method m) {
		return !m.equals(this.dummy);
	}
	
	public boolean isOptional(String id) {
		return id.indexOf("(OPT)") > -1;
	}
	
	public boolean isOptional(Place n) {
		return n.getId().indexOf("(OPT)") > -1;
	}
	
	public boolean isOptional(Transition t) {
		return false;
	}
	
	public boolean hasMethodWithNickname(String possibleNickName) {
		return this.methodAliases.containsKey(possibleNickName);
	}
	
	public boolean hasNicknamesOfMethod(Method m) {
		return this.aliasesForMethods.containsKey(m);
	}
	
	public boolean hasParticularNicknameOfMethod(Method m, String name) {
		return this.hasNicknamesOfMethod(m) && this.aliasesForMethods.get(m).contains(name);
	}
		
	public Method getMethodFromNickname(String nickname) {
		return this.methodAliases.get(nickname);
	}
	
	
	/**
	 * Retrieves all known aliases of a method. Methods may have several aliases
	 * due to factors like optional arguments.
	 * 
	 * Overloaded methods are simply regarded as separate objects, but since
	 * the method object itself is used as a key, this is fine.
	 * 
	 * @param m a Method whose nicknames we want
	 * @return		a list of the known aliases for the given method
	 */
	public ArrayList<String> getNicknamesOfMethod(Method m) {
		return this.aliasesForMethods.get(m);
	}
	
	/**
	 * Add an alias for a given method, so that we know what method the given
	 * string, which may be different, belongs to.
	 * 
	 * @param m					a Method who has a new nickname
	 * @param nickname		a String that is an alias for this method
	 */
	public void addMethodNickname(Method m, String nickname) {
		if (!this.methodAliases.containsKey(nickname)) {
			this.methodAliases.put(nickname, m);
		}
		if (!this.hasNicknamesOfMethod(m)) {
			this.aliasesForMethods.put(m, new ArrayList<String>());
		}
		ArrayList<String> nicknames = this.aliasesForMethods.get(m);
		if (!this.hasParticularNicknameOfMethod(m, nickname)) {
			// Just trying to verify that it does this...
			this.aliasesForMethods.get(m).add(nickname);
//			LOGGER.info("Before:");
//			LOGGER.info(this.aliasesForMethods.get(m));
//			nicknames.add(nickname);
//			LOGGER.info("After:");
//			LOGGER.info(this.aliasesForMethods.get(m));
		}
	}
	
	/**
	 * Add a clone transition to this MethodToPetriNet's PetriNet net, at place p,
	 * which should be a Place in the petri net.
	 * 
	 * @param p		a Place assumed to be inside of the PetriNet net
	 */
	public void addClone(Place p) {
		Transition t = this.createTransition(p.getId() + "_clone", dummy);
		this.net.createFlow(p, t);
		Flow f = this.net.createFlow(t, p);
		f.setWeight(2);
	}
	
	/**
	 * Getter for this object's petri net.
	 * 
	 * @return the petri net that this MethodsToPetriNet object has built
	 */
	public PetriNet getNet() {
		return this.net;
	}

	
	/**
	 * Puts in all of the transitions between parameter equivalent places in the
	 * petri net.
	 */
	private void addParamEquivalences() {
		Set<Place> places = this.net.getPlaces();
		Place[] placeArray = new Place [places.size()];
		places.toArray(placeArray);
		
		for (int i = 0; i < placeArray.length-1; i++) {
			for (int j = 0; j < placeArray.length; j++) {
				if (i != j) {					
					String aid = placeArray[i].getId();
					String bid = placeArray[j].getId();
					Type ta = new Type(this.removeOptSuffix(aid));
					Type tb = new Type(this.removeOptSuffix(bid));
					
					if (ta.asParamsEqual(tb)) {
						boolean orderer = ta.isConstValue || ta.isReference || MethodsToPetriNet.isOptional(net.getNode(aid));
						
						String source = (orderer) ? bid : aid;
						
						if (!source.endsWith("(OPT)")) {
							String target = (orderer) ? aid : bid;
							MethodsToPetriNet.addTransition(net, source, target);						
						}
					} else if (ta.valueTypeName.contentEquals("rclcpp::Node") && tb.valueTypeName.contentEquals("rcl_node_t") && (ta.isSharedPointer == tb.isSharedPointer)) {
						MethodsToPetriNet.addTransition(net, aid, bid);
					}
				}
			}
		}
	}

	/**
	 * Create a transition with a given name that represents the given method.
	 * 
	 * This also adds the transition's name as an "alias" for the method, so that
	 * a sequence of transitions can be translated back into a sequence of methods.
	 * 
	 * @param name		String, the desired name for this transition
	 * @param m			Method, the method that the transition returned will represent
	 * @return				Transition, the transition created with this name
	 */
	private Transition createTransition(String name, Method m) {
		Transition t = this.net.createTransition(name);
		this.addMethodNickname(m, name);
		return t;
	}

	/**
	 * Removes the suffix "(OPT)" from a given string. This is handy for looking
	 * at the names of many Place nodes.
	 * 
	 * @param name		the String to remove the string "(OPT)" from.
	 * @return				String, either the same string (if there was no string "(OPT)",
	 * 							or all of the string preceding the part with "(OPT)".
	 */
	private String removeOptSuffix(String name) {
		int index = name.indexOf("(OPT)");
		if (index < 0) {
			return name;
		}
		return name.substring(0, index);
	}

	/**
	 * Convert the methods of the given MethodsToPetriNet object into a PetriNet.
	 * 
	 * @param mt 			MethodsToPetriNet object that holds a list of methods,
	 * 								as well as different correspondences between a PetriNet and
	 * 								the names of its methods
	 * @param methods	ArrayList<Method> object that contains all of the methods
	 * 								to represent in the PetriNet
	 * @return					PetriNet representing the arguments and methods contained 
	 * 								in the given ArrayList
	 */
	public static PetriNet convert(MethodsToPetriNet mt, ArrayList<Method> methods) {
		PetriNet pn = new PetriNet();
		
		HashMap<String, Integer> nameCounts = new HashMap<String, Integer>();
		
		for (Method m : methods) {
			if (!nameCounts.containsKey(m.name)) {
				nameCounts.put(m.name,	0);
			}
			nameCounts.put(m.name, nameCounts.get(m.name) + 1);
			
			Transition t = MethodsToPetriNet.makeMethodTransition(m, pn, nameCounts);
			
			mt.addMethodNickname(m, t.getId());			
			
			String returnType = m.returnType.toString();
			Place ret = MethodsToPetriNet.getPetriPlace(pn, returnType);
			if (m.returnType.isSharedPointer) {
				String transName = m.returnType.toString() + "::Shared_to_unshared";
				mt.addMethodNickname(mt.de_ptr, transName);
				
				MethodsToPetriNet.addTransition(pn, returnType, transName, m.returnType.valueTypeName);
			}
			
			if (!m.hasOptionalArgs()) {
				if (m.isClassMethod) {
					String instanceType = m.fromClass.toString();
					
					Place inst = MethodsToPetriNet.getPetriPlace(pn, instanceType);
					pn.createFlow(inst, t);
				}
				MethodsToPetriNet.addTransitionFlowToPetri(pn, m.args, t, ret);
			} else {
				ArrayList<Arg> mandatory = m.getMandatoryArgs();
				if (m.isClassMethod) {
					String instType = m.fromClass.toString();
					try {
						mandatory.add(new Arg(instType));
					} catch (ArgParseException e) {
						LOGGER.info("ArgParseException occurred in MethodsToPetriNet.convert");
						LOGGER.info(e);
					}
				}
				
				MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, t, ret);
				
				ArrayList<Arg> options = m.getOptionalArgs();
				
				for (int i = 0; i < options.size(); i++) {
					Transition opt = MethodsToPetriNet.createAlternateTransition(pn, t, i);
					//pn.createTransition(t.getId() + "(ALT-" + Integer.toString(i) + ")");
					mt.addMethodNickname(m, opt.getId());
					
					// Add an additional arg that this transition will need to take
					mandatory.add(options.get(i));
					
					MethodsToPetriNet.addTransitionFlowToPetri(pn, mandatory, opt, ret);
				}	
			}
		}
		
		return pn;
	}

	/**
	 * Creates a dot file written using the DOT language in order to create a
	 * graphical representation of the given Petri Net.
	 * 
	 * @param pn	 	a PetriNet object to encode as a graph using DOT
	 */
	public static void createDotFile(PetriNet pn) {
		MethodsToPetriNet.createDotFile(pn, "dot/petri.dot");
	}
	
	public static void createDotFile(PetriNet pn, String fileName) {
		try {			
			FileWriter w = new FileWriter(fileName);
			
			HashMap<String, String> graphOptions = new HashMap<String, String>();
			graphOptions.put("rankdir", "LR");
			graphOptions.put("nodesep", "1");
			graphOptions.put("ranksep", "2");
			
			HashMap<String, String> colorEdgesFrom = new HashMap<String, String>();
			colorEdgesFrom.put("const std::string&", "red");
			colorEdgesFrom.put("CallbackT &&", "blue");
			colorEdgesFrom.put("bool(OPT)", "orange");
			
			w.write("digraph {\n");
			
			for (String key : graphOptions.keySet()) {
				w.write("\t" + key + "=" + graphOptions.get(key) + ";\n");
			}
			
			
			Set<Transition> trans = pn.getTransitions();
			
			for (Transition t : trans) {
				w.write("\t\"" + t.getId() + "\" [shape=box,style=filled,color=black,fontcolor=white];\n");
			}
						
			Set<Flow> flows = pn.getEdges();
			
			// Draw all of the edges
			for (Flow f : flows) {
				Node n = f.getSource();
				Node p = f.getTarget();
				
			  // Label the edges with their weights
				w.write("\"" + n.getId() + "\" -> \"" + p.getId() + "\" [label=\"" + Integer.toString(f.getWeight()) + "\"");
				
				if (MethodsToPetriNet.isOptional(n)) {
					// Make optional args' edges dotted.
					w.write(",style=dotted");
				}
				if (colorEdgesFrom.containsKey(n.getId())) {
					w.write(",color=" + colorEdgesFrom.get(n.getId()));
				}
				w.write("];\n");
			}
			
			w.write("}\n");
			
			w.flush();
			w.close();
		} catch (Exception e) {
			LOGGER.warn("Exception caught while attempting to write dot file in MethodsToPetriNet: {}}", e);
//			LOGGER.info(e);
		}
	}

	private static Transition createAlternateTransition(PetriNet pn, Transition t, int altNumber) {
		return pn.createTransition(t.getId() + "(ALT-"  + Integer.toString(altNumber) + ")");
	}

	/**
	 * Add a clone transition, and the appropriate edges, to the place p in the
	 * petri net pn.
	 * 
	 * @param pn		a PetriNet to add the clone transition in
	 * @param p		some Place in pn to add the clone transition to
	 */
	private static void addClone(PetriNet pn, Place p) {
		Transition t = pn.createTransition(p.getId() + "_clone");
		pn.createFlow(p, t);
		Flow f = pn.createFlow(t, p);
		f.setWeight(2);
	}
	
	/**
	 * Get the Place node from a petri net with a given id, if it exists.
	 * 
	 * If it doesn't, create that Place, and then return it.
	 * 
	 * @param pn		the PetriNet to look for the id in
	 * @param id		String, the desired id of the Place
	 * @return			Place, one of the nodes representing a type in the Petri Net
	 */
	private static Place getPetriPlace(PetriNet pn, String id) {
		return (!pn.containsPlace(id)) ? pn.createPlace(id) : pn.getPlace(id);
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
			LOGGER.warn("Flow already existed, but will increase weight of flow:", e);
//			LOGGER.info(e);
			
			try {
				Flow f = pn.getFlow(p.getId(), t.getId());
				int wt = f.getWeight();
				
//				LOGGER.info("Increasing weight of flow (" + p.getId() + ", " + t.getId() + ") from " + Integer.toString(wt) + " to " + Integer.toString(wt + 1));
				
				// Increase the weight
				f.setWeight(wt + 1);
			} catch (NoSuchEdgeException e1) {
				LOGGER.warn("Caught exception in MethodsToPetriNet.addFlowToPetri:", e1);
//				LOGGER.info(e1);
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
