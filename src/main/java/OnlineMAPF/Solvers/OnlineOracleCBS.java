package OnlineMAPF.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.*;
import BasicCBS.Solvers.AStar.DistanceTableAStarHeuristic;
import BasicCBS.Solvers.AStar.RunParameters_SAAStar;
import BasicCBS.Solvers.CBS.CBS_Solver;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.I_ConflictManager;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.NaiveConflictDetection;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import Environment.Metrics.InstanceReport;
import OnlineMAPF.*;

import java.util.Comparator;

/**
 * An version of {@link CBS_Solver CBS} where agents have arrival times.
 *
 * Agents disappear at their goal, and start at a private garage. Solves optimally, knowing the future arrival times of
 * all agents in advance. This mean that this solver is actually offline, solving the entire problem at once.
 */
public class OnlineOracleCBS extends CBS_Solver {

    public OnlineOracleCBS() {
        // use online aStar.
        super(new OnlineAStar(), null, null, null, null);
    }

    @Override
    protected void init(MAPF_Instance instance, RunParameters runParameters) {
        super.init(instance, runParameters);
        // verify that all agents are online
        for (Agent agent :
                instance.agents) {
            if (! (agent instanceof OnlineAgent) )
                throw new IllegalArgumentException(this.getClass().getSimpleName() + " is an online solver and accepts only Online Agents.");
        }
        // convert the heuristic to an online heuristic
        if(super.aStarHeuristic != null && super.aStarHeuristic instanceof DistanceTableAStarHeuristic){
            super.aStarHeuristic = new OnlineDistanceTableAStarHeuristic((DistanceTableAStarHeuristic)super.aStarHeuristic);
        }
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        return new OnlineSolution(super.runAlgorithm(instance, parameters));
    }

    /**
     * Use online Conflict manager
     * @param node {@inheritDoc}
     * @return {@inheritDoc}
     */
    @Override
    protected I_ConflictManager getConflictAvoidanceTableFor(CBS_Solver.CBS_Node node) {
//        I_ConflictManager cat = new OnlineConflictManager();
        I_ConflictManager cat = new NaiveConflictDetection(false);
        for (SingleAgentPlan plan :
                node.getSolution()) {
            cat.addPlan(plan);
        }
        return cat;
    }

    @Override
    protected RunParameters getSubproblemParameters(Solution currentSolution, ConstraintSet constraints, InstanceReport instanceReport, MAPF_Instance subproblem) {
        RunParameters parameters = super.getSubproblemParameters(currentSolution, constraints, instanceReport, subproblem);

        // convert the constraint set to an online constraint set.
        parameters.constraints = new OnlineConstraintSet(constraints);

        Agent agent = subproblem.agents.get(0);
        // assumes agents are online agents, and throws an exception if they aren't
        OnlineAgent onlineAgent = ((OnlineAgent) agent);
        // requires an online low level solver. the only one currently implemented is OnlineAStar
        RunParameters_SAAStar astarParameters = ((RunParameters_SAAStar)parameters);

        // set start time for when the agent arrives
        astarParameters.problemStartTime = onlineAgent.arrivalTime;

        // set the agent to start at its private garage
        astarParameters.agentStartLocation = ((OnlineAgent) agent).getPrivateGarage(subproblem.map.getMapCell(agent.source));

        return astarParameters;
    }
}
