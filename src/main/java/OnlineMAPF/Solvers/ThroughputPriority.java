package OnlineMAPF.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Instances.Maps.I_Location;
import BasicCBS.Solvers.AStar.RunParameters_SAAStar;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicCBS.Solvers.I_Solver;
import BasicCBS.Solvers.Move;
import BasicCBS.Solvers.SingleAgentPlan;
import BasicCBS.Solvers.Solution;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;
import OnlineMAPF.OnlineConstraintSet;

import java.util.*;

public class ThroughputPriority extends OnlineCBSSolver{

    public ThroughputPriority() {
        super.name = "ThroughputPriority";
    }

    public ThroughputPriority(boolean preserveSolutionsInNewRoots) {
        super(preserveSolutionsInNewRoots);
        super.name = "ThroughputPriority";
    }

    @Override
    protected Solution solveForNewArrivals(int time, HashMap<Agent, I_Location> currentAgentLocations) {
        // contains only the new agents
        OnlineAStar onlineAStar = new OnlineAStar(costOfReroute);
        Solution newSolution = new Solution();
        OnlineConstraintSet constraints = new OnlineConstraintSet();
        // create a list of all agents in the problem (existing and haven't finished or new) that is sorted by individual
        // path length. solve such that agents receive priority by who has the shortest free-space-assumption path from
        // current location to goal
        List<Agent> agentsByPathLength = getAgentsByPathLength(currentAgentLocations, baseInstance, time);
        for (Agent agent : agentsByPathLength){
            // reduce the problem to just the agent
            MAPF_Instance subProblem = baseInstance.getSubproblemFor(agent);
            RunParameters_SAAStar astarParameters = new RunParameters_SAAStar(timeoutThreshold - totalRuntime, constraints,
                    S_Metrics.newInstanceReport(), null, null, time);
            astarParameters.agentStartLocation = currentAgentLocations.get(agent);
            Solution newAgentSolution = onlineAStar.solve(subProblem, astarParameters);
            if (newAgentSolution == null) {
                return null;
            }
            newSolution.putPlan(newAgentSolution.getPlanFor(agent));
            // add the constraints protecting the planned agent
            constraints.addAll(allConstraintsForSolution(newAgentSolution));
            digestSubproblemReport(astarParameters.instanceReport);
        }
        latestSolution = newSolution;
        return latestSolution;
    }

    private List<Agent> getAgentsByPathLength(HashMap<Agent, I_Location> currentAgentLocations, MAPF_Instance baseInstance, int time) {
        List<Agent> sortedAgents = new ArrayList<>(currentAgentLocations.keySet());
        I_Solver solver = new OnlineAStar();
        RunParameters_SAAStar astarParameters = new RunParameters_SAAStar(timeoutThreshold - totalRuntime, null,
                new InstanceReport(), null, null, time);
        sortedAgents.sort(Comparator.comparing(  agent -> {
                    astarParameters.agentStartLocation = currentAgentLocations.get(agent);
                    return solver.solve(baseInstance.getSubproblemFor(agent), astarParameters).getPlanFor(agent).size();
                }));
        return sortedAgents;
    }

    protected List<Constraint> allConstraintsForSolution(Solution solution) {
        List<Constraint> constraints = new LinkedList<>();
        for( SingleAgentPlan plan : solution){
            // protect the agent's plan
            for (Move move :
                    plan) {
                constraints.add(move.vertexConstraintsForMove(null));
                constraints.add(move.swappingConstraintsForMove(null));
            }
        }
        return constraints;
    }

    @Override
    public String name() {
        return super.name;
    }

}
