package OnlineMAPF;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.Move;
import BasicCBS.Solvers.SingleAgentPlan;
import BasicCBS.Solvers.Solution;

import java.util.*;

/**
 * A solution to an online problem.
 *
 * Contains the solutions that were returned by the solver at different times. Also contains (at {@link Solution}) a merged
 * solution, representing just the paths that agents ended up following.
 */
public class OnlineSolution extends Solution{

    public final SortedMap<Integer, Solution> solutionsAtTimes;

    public OnlineSolution(SortedMap<Integer, Solution> solutionsAtTimes) {
        //make unified solution for super
        super(mergeSolutions(solutionsAtTimes));

        this.solutionsAtTimes = solutionsAtTimes;
        super.checkGoalInValidation = false;
    }

    public OnlineSolution(Solution offlineSolution) {
        this(putOfflineSolutionInMap(offlineSolution));
    }

    private static SortedMap<Integer, Solution> putOfflineSolutionInMap(Solution offlineSolution){
        SortedMap<Integer, Solution> solutionsAtTimes = new TreeMap<>();
        solutionsAtTimes.put(0, offlineSolution);
        return solutionsAtTimes;
    }

    /**
     * Merge the solutions that the solver produced at different times into one solution, which represents the paths that
     * the agents actually ended up taking.
     * @param solutionsAtTimes solutions that the solver produced at different times
     * @return a merged solution
     */
    public static Map<Agent, SingleAgentPlan> mergeSolutions(SortedMap<Integer, Solution> solutionsAtTimes) {
        Map<Agent, SingleAgentPlan> mergedAgentPlans = new HashMap<>();
        // for every time where new agents arrived (and so the existing plans were changed)
        for (int time :
                solutionsAtTimes.keySet()) {
            Solution solution = solutionsAtTimes.get(time);
            // every agent included in this solution. meaning it arrived at/before time (the solution's start time), and
            // hasn't reached its goal yet.
            for (SingleAgentPlan plan :
                    solution) {
                Agent agent = plan.agent;
                if(! mergedAgentPlans.containsKey(agent)){ // agent arrived at time
                    mergedAgentPlans.put(agent, plan);
                }
                else{ // agent was already around, merge the plans to represent what it actually ended up doing.
                    SingleAgentPlan previousPlan = mergedAgentPlans.get(agent);
                    SingleAgentPlan newPlan = solution.getPlanFor(agent);
                    SingleAgentPlan mergedPlan = mergePlans(previousPlan, newPlan);
                    mergedAgentPlans.put(agent, mergedPlan);
                }
            }
        }
        return mergedAgentPlans;
    }

    /**
     * Merges two {@link SingleAgentPlan plans}.
     * @param oldPlan keeps the moves from this plan that don't overlap with the other plan.
     * @param newPlan keeps the moves from this plan that are newer (larger time value) than all moves in the other plan.
     * @return a merged plan.
     */
    private static SingleAgentPlan mergePlans(SingleAgentPlan oldPlan, SingleAgentPlan newPlan){
        SingleAgentPlan merged = new SingleAgentPlan(oldPlan.agent);
        // if the new plan is empty (agent had just arrived at its goal at the time of the new plan's start)
        if(newPlan.size() == 0){
            return oldPlan;
        }
        else {
            // add every move before the time where the new plan starts
            for (int time = oldPlan.getFirstMoveTime(); time < newPlan.getFirstMoveTime(); time++) {
                merged.addMove(oldPlan.moveAt(time));
            }
            // add all of the moves in the new plan
            for (Move move :
                    newPlan) {
                merged.addMove(move);
            }
            return merged;
        }
    }

    @Override
    protected boolean planStartsAtSourceEndsAtTarget(SingleAgentPlan plan, MAPF_Instance instance) {
        return plan.size() == 0 || (
                plan.moveAt(plan.getFirstMoveTime()).prevLocation.equals(     /*start at source*/
                ((OnlineAgent)(plan.agent)).getPrivateGarage(instance.map.getMapCell(plan.agent.source))) /*convert to online (private garage)*/
                && plan.moveAt(plan.getEndTime()).currLocation.equals(instance.map.getMapCell(plan.agent.target)) )/*end at target*/;
    }

    public int numReroutes(){
        int numReroutes = 0;
        Solution prevSolution = null;
        // for every time where new agents arrived (and so the existing plans were possibly changed)
        for (int time :
                solutionsAtTimes.keySet()) {
            Solution solution = solutionsAtTimes.get(time);
            // every agent included in this solution. meaning it arrived at/before time (the solution's start time), and
            // hasn't reached its goal yet.
            if(prevSolution != null){
                numReroutes += numReroutes(prevSolution, solution);
            }
            prevSolution = solution;
        }
        return numReroutes;
    }

    /**
     * Counts the number reroutes to get from the existing solution to the new solution. That is, the number of plans
     * in the new solution that are different from the corresponding plan in the existing solution, starting from the
     * start time of the new solution.
     * @param existingSolution the existing (old) solution.
     * @param newSolution the new solution.
     * @return the number of reroutes in the new solution.
     */
    public static int numReroutes(Solution existingSolution, Solution newSolution){
        int numReroutes = 0;
        for (SingleAgentPlan plan :
                newSolution) {
            Agent agent = plan.agent;
            if(existingSolution.getPlanFor(agent) != null){ // agent already had a plan
                SingleAgentPlan previousPlan = existingSolution.getPlanFor(agent);
                SingleAgentPlan newPlan = newSolution.getPlanFor(agent);
                //check if the plan changed and count it as a reroute if it did
                if(previousPlan.getEndTime() != newPlan.getEndTime()){
                    numReroutes++;
                }
                else {
                    for (int t = newPlan.getFirstMoveTime(); t < newPlan.getEndTime(); t++) {
                        if( isAReroute(previousPlan, newPlan.moveAt(t)) ){
                            numReroutes++;
                            break;
                        }
                    }
                }
            }
        }
        return numReroutes;
    }

    /**
     * Returns true if the given move is different from the planned move at that time, meaning a reroute.
     * @param previousPlan - a previously existing plan.
     * @param move a move an agent may want to make in a new plan that is being considered.
     * @return true if the given move is different from the planned move at that time, meaning a reroute.
     */
    public static boolean isAReroute(SingleAgentPlan previousPlan, Move move) {
        return ! Objects.equals(previousPlan.moveAt(move.timeNow), move);
    }

    /**
     * The cost of all reroutes that happened in the online solution.
     * @return the cost of all reroutes that happened in the online solution
     */
    public int costOfReroutes(int costOfSingleReroute) {
        return numReroutes() * costOfSingleReroute;
    }

    /**
     * The cost of all the reroutes between an old and a new solution.
     * @param existingSolution -
     * @param existingSolution the existing (old) solution.
     * @param newSolution the new solution.
     * @return the cost of reroutes in the new solution.
     */
    public static int costOfReroutes(Solution existingSolution, Solution newSolution, int costOfSingleReroute){
        return costOfSingleReroute * numReroutes(existingSolution, newSolution);
    }

}
