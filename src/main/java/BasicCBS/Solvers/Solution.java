package BasicCBS.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.ConstraintsAndConflicts.SwappingConflict;
import BasicCBS.Solvers.ConstraintsAndConflicts.VertexConflict;

import java.util.*;
import java.util.function.Consumer;

/**
 * A collection of {@link SingleAgentPlan}s, representing a solution to a Path Finding problem.
 * If the collection contains more than one plan, it is a solution to a Multi Agent Path Finding problem.
 */
public class Solution implements Iterable<SingleAgentPlan>{
    /**
     * A {@link Map}, mapping {@link Agent agents} to their {@link SingleAgentPlan plans}.
     */
    protected final Map<Agent, SingleAgentPlan> agentPlans;
    protected boolean checkGoalInValidation = true;

    public Solution(Map<Agent, SingleAgentPlan> agentPlans) {
        this.agentPlans = new HashMap<>(agentPlans);
    }

    public Solution(Iterable<? extends SingleAgentPlan> plans) {
        if(plans instanceof Solution){
            Solution sol = ((Solution)plans);
            this.agentPlans = new HashMap<>(sol.agentPlans);
        }
        else{
            Map<Agent, SingleAgentPlan> agentPlanMap = new HashMap<>();
            for (SingleAgentPlan plan :
                    plans) {
                agentPlanMap.put(plan.agent, plan);
            }
            this.agentPlans = agentPlanMap;
        }
    }

    public Solution(){
        this(new HashMap<>());
    }

    public SingleAgentPlan getPlanFor(Agent agent){
        return agentPlans.get(agent);
    }

    public SingleAgentPlan putPlan(SingleAgentPlan singleAgentPlan){
        if(singleAgentPlan == null) {throw new IllegalArgumentException();}
        return this.agentPlans.put(singleAgentPlan.agent, singleAgentPlan);
    }

    /**
     * Looks for vertex conflicts ({@link VertexConflict}) or swapping conflicts ({@link SwappingConflict}). Runtime is
     * O( (n-1)*mTotal ) , where n = the number of {@link SingleAgentPlan plans}/{@link Agent agents} in this solution,
     * and mTotal = the total number of moves in all plans together.
     * @return true if the solution is valid (contains no vertex or swapping conflicts).
     */
    public boolean isValidSolution(){
        List<SingleAgentPlan> allPlans = new ArrayList<>(agentPlans.values());
        for (int i = 0; i < allPlans.size(); i++) {
            SingleAgentPlan plan1 = allPlans.get(i);
            for (int j = i+1; j < allPlans.size(); j++) {
                SingleAgentPlan plan2 = allPlans.get(j);
                if(plan1.conflictsWith(plan2, checkGoalInValidation)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Validates that this solution is a valid solution for the given {@link MAPF_Instance}. Where
     * {@link #isValidSolution()} only validates that this solution is free of conflicts, {@link #solves(MAPF_Instance)}
     * also validates that:
     * 1. The solution covers every {@link Agent} with a {@link SingleAgentPlan}.
     * 2. The solution doesn't cover agents not present in the instance.
     * 3. The plan for each agent starts at its source and ends at its target.
     * 4. The times of moves in plans are consistent - always increase by 1.
     * 5. The locations of moves in the plans are consistent - agents always move from a vertex to its neighbor.
     * These extra checks make this validation more expensive, but it is useful for debugging purposes.
     * @param instance an {@link MAPF_Instance} that this solution supposedly solves.
     * @return boolean if this solution solves the instance.
     */
    public boolean solves(MAPF_Instance instance){
        // check that the solution is conflict free
        if (!isValidSolution())
            return false;
        // check that the solution covers all agents and no other agents
        if (!this.agentPlans.keySet().containsAll(instance.agents) || !instance.agents.containsAll(this.agentPlans.keySet()))
            return false;
        for (SingleAgentPlan plan :
                agentPlans.values()) {
            if(plan.size() == 0) continue;
            // check start and end at source and target
            if (!planStartsAtSourceEndsAtTarget(plan, instance)){
                return false;
            }
            // agents always move from a vertex to its neighbor
            Move prevMove = plan.moveAt(plan.getFirstMoveTime());
            // check that the move is internally consistent
            if (!(prevMove.prevLocation.isNeighbor(prevMove.currLocation) || prevMove.prevLocation.equals(prevMove.currLocation)))
                return false;
            for (int time = plan.getFirstMoveTime() + 1; time <= plan.getEndTime(); time++) {
                Move currMove = plan.moveAt(time);
                // check that the move is internally consistent
                if (!(currMove.prevLocation.isNeighbor(currMove.currLocation) || currMove.prevLocation.equals(currMove.currLocation)))
                    return false;
                // check that the move is consistent with the next move
                if (!prevMove.currLocation.equals(currMove.prevLocation))
                    return false;
                // check that the time of the move is consistent with the next move
                if (currMove.timeNow-prevMove.timeNow != 1)
                    return false;
                prevMove = currMove;
            }
        }
        return true;
    }

    protected boolean planStartsAtSourceEndsAtTarget(SingleAgentPlan plan, MAPF_Instance instance){
        return plan.moveAt(plan.getFirstMoveTime()).prevLocation.equals(instance.map.getMapCell(plan.agent.source)) /*start at source*/
                && plan.moveAt(plan.getEndTime()).currLocation.equals(instance.map.getMapCell(plan.agent.target)) /*end at target*/;
    }

    /**
     * The number of plans (agents) in the solution.
     * @return the number of plans (agents) in the solution.
     */
    public int size(){
        return agentPlans.size();
    }

    public int sumIndividualCosts(){
        int SOC = 0;
        for (SingleAgentPlan plan :
                agentPlans.values()) {
            SOC += plan.getCost();
        }
        return SOC;
    }

    public int makespan(){
        int maxCost = 0;
        for (SingleAgentPlan plan :
                agentPlans.values()) {
            maxCost = Math.max(maxCost, plan.getCost());
        }
        return maxCost;
    }

    @Override
    public String toString() {
        return this.readableToString();
    }

    //nicetohave JSON toString

    /**
     * A string output that is easier for humans to read.
     * @return a string output that is easier for humans to read.
     */
    public String readableToString(){
        StringBuilder sb = new StringBuilder();
        List<Agent> agents = new ArrayList<>(this.agentPlans.keySet());
        Collections.sort(agents, Comparator.comparing(agent -> agent.iD));
        for(Agent agent : agents){
            sb.append("Plan for agent ").append(agent.iD);
            for(Move move : this.agentPlans.get(agent)){
                sb.append('\n').append(move.timeNow).append(": ").append(move.prevLocation.getCoordinate()).append(" -> ").append(move.currLocation.getCoordinate());
            }
            sb.append("\n");
        }
        sb.append('\n');
        return sb.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Solution that = (Solution) o;
        return agentPlans.equals(that.agentPlans);
    }

    @Override
    public int hashCode() {
        return Objects.hash(agentPlans);
    }

    /*  = Iterator Interface =  */

    @Override
    public Iterator<SingleAgentPlan> iterator() {
        return agentPlans.values().iterator();
    }

    @Override
    public void forEach(Consumer<? super SingleAgentPlan> action) {
        agentPlans.values().forEach(action);
    }

    @Override
    public Spliterator<SingleAgentPlan> spliterator() {
        return agentPlans.values().spliterator();
    }
}
