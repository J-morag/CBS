package OnlineMAPF.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Instances.Maps.I_Location;
import BasicCBS.Solvers.AStar.RunParameters_SAAStar;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicCBS.Solvers.Move;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.SingleAgentPlan;
import BasicCBS.Solvers.Solution;
import Environment.Metrics.S_Metrics;
import OnlineMAPF.OnlineAgent;
import OnlineMAPF.OnlineConstraintSet;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public class ReplanSingleGrouped extends OnlineCBSSolver {

    /**
     * Replan single (not grouped)
     */
    public boolean replanSingle;

    public ReplanSingleGrouped(boolean replanSingle) {
        this.replanSingle = replanSingle;
        super.name = replanSingle ? "ReplanSingle" : "ReplanSingleGrouped";
    }

    public ReplanSingleGrouped(boolean preserveSolutionsInNewRoots, boolean replanSingle) {
        super(preserveSolutionsInNewRoots);
        this.replanSingle = replanSingle;
        super.name = replanSingle ? "ReplanSingle" : "ReplanSingleGrouped";
    }

    @Override
    public Solution newArrivals(int time, List<? extends OnlineAgent> agents) {
        HashMap<Agent, I_Location> currentAgentLocations = new HashMap<>(agents.size());
        // new agents will start at their private garages.
        addNewAgents(agents, currentAgentLocations);

        return solveForNewArrivals(time, currentAgentLocations);
    }

    @Override
    protected Solution solveForNewArrivals(int time, HashMap<Agent, I_Location> currentAgentLocations) {
        // contains only the new agents
        OnlineAStar onlineAStar = new OnlineAStar(costOfReroute);
        Solution solutionForNewAgents = new Solution();
        if (replanSingle){
            // protect existing agents with constraints
            OnlineConstraintSet constraints = new OnlineConstraintSet(allConstraintsForSolution(latestSolution));
            // currentAgentLocations has been reduced to just the new agents
            for (Agent agent : currentAgentLocations.keySet()){
                // reduce the problem to just the new agent
                MAPF_Instance subProblem = baseInstance.getSubproblemFor(agent);
                RunParameters_SAAStar astarParameters = new RunParameters_SAAStar(timeoutThreshold - totalRuntime, constraints,
                        S_Metrics.newInstanceReport(), null, null, time);
                astarParameters.agentStartLocation = currentAgentLocations.get(agent);
                Solution newAgentSolution = onlineAStar.solve(subProblem, astarParameters);
                if (newAgentSolution == null) {
                    return null;
                }
                solutionForNewAgents.putPlan(newAgentSolution.getPlanFor(agent));
                // add the constraints protecting the new agent
                constraints.addAll(allConstraintsForSolution(newAgentSolution));
                digestSubproblemReport(astarParameters.instanceReport);
            }
        }
        else { // replan single grouped
            // protect existing agents with constraints
            ConstraintSet constraints = new ConstraintSet(allConstraintsForSolution(latestSolution));
            OnlineCompatibleOfflineCBS offlineSolver = new OnlineCompatibleOfflineCBS(currentAgentLocations, time, null, onlineAStar);
            for (Agent agent : currentAgentLocations.keySet()){
                // reduce the problem to just the new agent
                MAPF_Instance subProblem = baseInstance.getSubproblemFor(agent);
                RunParameters runParameters = new RunParameters(timeoutThreshold - totalRuntime, constraints,
                        S_Metrics.newInstanceReport(), null);
                Solution newAgentSolution = offlineSolver.solve(subProblem, runParameters);
                if (newAgentSolution == null) {
                    return null;
                }
                solutionForNewAgents.putPlan(newAgentSolution.getPlanFor(agent));
                // add the constraints protecting the new agent
                constraints.addAll(allConstraintsForSolution(newAgentSolution));
                digestSubproblemReport(runParameters.instanceReport);
            }
        }
        // add the remaining parts of the plans of existing agents
        for(SingleAgentPlan existingAgentPlan : latestSolution){
            if(existingAgentPlan.getEndTime() >= time){
                SingleAgentPlan trimmedPlan = new SingleAgentPlan(existingAgentPlan.agent);
                for (int t = time; t <= existingAgentPlan.getEndTime(); t++) {
                    trimmedPlan.addMove(existingAgentPlan.moveAt(t));
                }
                solutionForNewAgents.putPlan(trimmedPlan);
            }
        }
        latestSolution = solutionForNewAgents;
        return latestSolution;
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
