package ros2sy.logic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.sat4j.core.VecInt;
//import org.sat4j.minisat.SolverFactory;
//import org.sat4j.specs.ISolver;
import org.sat4j.pb.IPBSolver;
import org.sat4j.pb.SolverFactory;
import org.sat4j.specs.ContradictionException;
import org.sat4j.specs.TimeoutException;

public class SATSolver {

	private IPBSolver solver = null;
	private boolean unsat = false;
	private VecInt assumptions;

	enum ConstraintType {
		LTE, EQ, GTE;
	}

	// Maps the variable id to transition
	public HashMap<Integer, Variable> id2variable = new HashMap<>();

	private int nbVariables = 0;
	public VecInt loc_variables;

	private VecInt objective = new VecInt();
	private VecInt coeffs = new VecInt();
	private List<String> names = new ArrayList<>();

	public SATSolver() {
		solver = SolverFactory.newDefault();
		assumptions = new VecInt();
		loc_variables = new VecInt();
	}

	public void reset() {
		solver = SolverFactory.newDefault();
		unsat = false;
		id2variable.clear();
		nbVariables = 0;
		loc_variables.clear();
		objective.clear();
		coeffs.clear();
		names.clear();
	}

	public void setObjective(VecInt obj, VecInt c, List<String> names) {
		obj.copyTo(objective);
		c.copyTo(coeffs);
		this.names.addAll(names);
	}

	public int getNbConstraints() {
		return solver.nConstraints();
	}

	public void setNbVariables(int vars) {
		
		int extra_vars = 100;

		// version for additional variables
		for (int i = vars + 1; i <= vars + extra_vars; i++)
			loc_variables.push(i);
		nbVariables = vars + extra_vars;
		solver.newVar(nbVariables + extra_vars);

		// dummy constraints for the additional variables
		// each variable much appear at least once in the solver
		for (int i = vars + 1; i <= extra_vars; i++) {
			try {
				solver.addAtLeast(new VecInt(new int[] { i }), 1);
			} catch (ContradictionException e) {
				assert (false);
			}
		}

//		nbVariables = vars;
//		solver.newVar(nbVariables);
	}

	public int getNbVariables() {
		return nbVariables;
	}

	public void addClause(VecInt constraint) {
		try {
			solver.addClause(constraint);
		} catch (ContradictionException e) {
			unsat = false;
		}
	}

	public void addConstraint(VecInt constraint, VecInt coeffs, ConstraintType ct, int k) {
		try {
			switch (ct) {
			case LTE:
				solver.addAtMost(constraint, coeffs, k);
				break;
			case EQ:
				solver.addExactly(constraint, coeffs, k);
				break;
			case GTE:
				solver.addAtLeast(constraint, coeffs, k);
				break;
			default:
				assert (false);
			}
		} catch (ContradictionException e) {
			unsat = true;
		}
	}

	public void addConstraint(VecInt constraint, ConstraintType ct, int k) {
		try {
			switch (ct) {
			case LTE:
				solver.addAtMost(constraint, k);
				break;
			case EQ:
				solver.addExactly(constraint, k);
				break;
			case GTE:
				solver.addAtLeast(constraint, k);
				break;
			default:
				assert (false);
			}
		} catch (ContradictionException e) {
			unsat = true;
		}
	}

	public void setAssumption(int v) {
		assumptions.push(v);
	}

	public void setTrue(int v) {
		try {
			VecInt clause = new VecInt(new int[] { v });
			solver.addClause(clause);
		} catch (ContradictionException e) {
			unsat = true;
		}
	}

	public void setFalse(int v) {
		try {
			VecInt clause = new VecInt(new int[] { -v });
			solver.addClause(clause);
		} catch (ContradictionException e) {
			unsat = true;
		}
	}

	public int getCost() {

		int[] model = solver.model();
		if (model.length == 0)
			return -1;

		int cost = 0;
		for (int i = 0; i < objective.size(); i++) {
			if (model[objective.get(i) - 1] > 0) {
				cost += coeffs.get(i);
//				System.out.println("api = " + names.get(i));
			}
		}
		return cost;
	}

	public List<Variable> findPath(int loc) {

		ArrayList<Variable> res = new ArrayList<>();
		try {
			// comment the below assert when using assumptions
			assert (assumptions.isEmpty());
			if (!unsat && solver.isSatisfiable(assumptions)) {
				int[] model = solver.model();
				assert (model.length == nbVariables);
				VecInt block = new VecInt();
				for (Integer id : id2variable.keySet()) {
					if (model[id - 1] > 0) {
						block.push(-id);
						res.add(id2variable.get(id));
					}
				}

				// block model
				try {
					solver.addClause(block);
				} catch (ContradictionException e) {
					unsat = true;
				}

			}
		} catch (TimeoutException e) {
			// consider as it did not find a solution
			unsat = true;
		}

		// sort transitions by increasing time step
		Collections.sort(res);

		return res;
	}
}
