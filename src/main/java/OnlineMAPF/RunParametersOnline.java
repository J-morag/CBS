package OnlineMAPF;

import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.Solution;
import Environment.Metrics.InstanceReport;

/**
 * Run parameters for an Online MAPF solver
 */
public class RunParametersOnline extends RunParameters {
    /**
     * A non negative cost for rerouting an agent during an online solution.
     */
    public int costOfReroute = 0;

    /**
     * The maximum allowed number of waiting agents
     */
    public int agentQueueCapacity = 0;
    /**
     * The maximum number of agents allowed concurrently in the problem space.
     */
    public int problemSpaceAgentCapacity = 10000;

    public RunParametersOnline(long timeout, ConstraintSet constraints, InstanceReport instanceReport, Solution existingSolution, int costOfReroute) {
        super(timeout, constraints, instanceReport, existingSolution);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(ConstraintSet constraints, InstanceReport instanceReport, Solution existingSolution, int costOfReroute) {
        super(constraints, instanceReport, existingSolution);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(ConstraintSet constraints, InstanceReport instanceReport, int costOfReroute) {
        super(constraints, instanceReport);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(InstanceReport instanceReport, Solution existingSolution, int costOfReroute) {
        super(instanceReport, existingSolution);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(InstanceReport instanceReport, int costOfReroute) {
        super(instanceReport);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(ConstraintSet constraints, int costOfReroute) {
        super(constraints);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(Solution existingSolution, int costOfReroute) {
        super(existingSolution);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(long timeout, int costOfReroute) {
        super(timeout);
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline(int costOfReroute) {
        this.costOfReroute = costOfReroute;
    }

    public RunParametersOnline() {
    }
}
