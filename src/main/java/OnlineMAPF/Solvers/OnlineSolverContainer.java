package OnlineMAPF.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Instances.Maps.I_Location;
import BasicCBS.Solvers.*;
import Environment.Metrics.InstanceReport;
import OnlineMAPF.OnlineAgent;
import OnlineMAPF.OnlineSolution;
import OnlineMAPF.RunParametersOnline;

import java.util.*;

/**
 * Simulates an online environment for an online solver to run in.
 *
 * New {@link OnlineAgent agents} arrive at different times, and the {@link #onlineSolver} has to accommodate them into
 * a new {@link Solution}, which it then returns.
 */
public class OnlineSolverContainer implements I_Solver {

    /**
     * An online solver to use for solving online problems.
     */
    private final I_OnlineSolver onlineSolver;
    /**
     * A custom arrival time to give {@link Agent offline agents} that this solver attempts to solve for.
     * Defaults to {@link OnlineAgent#DEFAULT_ARRIVAL_TIME}.
     */
    private static final int ARRIVAL_TIME_FOR_OFFLINE_AGENTS = OnlineAgent.DEFAULT_ARRIVAL_TIME;
    /**
     * The cost of rerouting an agent.
     */
    private int costOfReroute;
    /**
     * A queue of agents waiting to enter.
     */
    private Queue<OnlineAgent> agentQueue;
    /**
     * The maximum allowed number of waiting agents
     */
    private int agentQueueCapacity;
    /**
     * The maximum number of agents allowed concurrently in the problem space.
     */
    private int problemSpaceAgentCapacity;
    private boolean breachedCapacity;

    public OnlineSolverContainer(I_OnlineSolver onlineSolver) {
        if(onlineSolver == null) {
            throw new IllegalArgumentException("null is not an acceptable value for onlineSolver");
        }
        this.onlineSolver = onlineSolver;
    }

    public void init(MAPF_Instance instance, RunParameters runParameters){
        verifyAgentsUniqueId(instance.agents);

        this.costOfReroute = runParameters instanceof RunParametersOnline ? ((RunParametersOnline)runParameters).costOfReroute : 0;
        // must initialize the solver because later we will only be giving it new agents, no other data
        this.onlineSolver.setEnvironment(instance, runParameters);
        // check if there is there is a maximum capacity for the problem space or the queue
        this.agentQueue = new LinkedList<>();
        if (runParameters instanceof  RunParametersOnline){
            RunParametersOnline parametersOnline = ((RunParametersOnline)runParameters);
            this.agentQueueCapacity = Math.max(parametersOnline.agentQueueCapacity, 0);
            this.problemSpaceAgentCapacity = parametersOnline.problemSpaceAgentCapacity > 0 ? parametersOnline.problemSpaceAgentCapacity : 10000;
        }
        else{
            this.agentQueueCapacity = 0;
            this.problemSpaceAgentCapacity = 10000;
        }
        this.breachedCapacity = false;
    }

    /**
     * Runs a simulation of an online environment. New {@link OnlineAgent agents} arrive at different times, and the
     * {@link #onlineSolver} has to accommodate them into a new {@link Solution}, which it then returns.
     * Finally, the solutions are combined into an {@link OnlineSolution}.
     * @param instance a problem instance to solve.
     * @param parameters parameters that expand upon the problem instance or change the solver's behaviour for this specific
     *                   run.
     * @return an {@link OnlineSolution}.
     */
    @Override
    public Solution solve(MAPF_Instance instance, RunParameters parameters) {
        init(instance, parameters);

        SortedMap<Integer, Solution> solutionsAtTimes = new TreeMap<>();
        SortedMap<Integer, List<OnlineAgent>> agentsForTimes = OnlineSolverContainer.getAgentsByTime(instance.agents);

        // feed the solver with new agents for every timestep when new agents arrive, or when space becomes available and there are agents in queue
        // agentsForTimes should be a sorted map
        while (! agentsForTimes.isEmpty()) {
            int timestepWithNewAgents = agentsForTimes.firstKey();
            // have to copy the list to avoid having two iterators on agentsForTimes open, and throwing
            // java.util.ConcurrentModificationException when adding to queue
            List<OnlineAgent> newArrivals = new ArrayList<>(agentsForTimes.get(timestepWithNewAgents));
            // add the new agents to the queue
            this.agentQueue.addAll(newArrivals);
            int agentsInProblem = agentsInProblem(timestepWithNewAgents, solutionsAtTimes);
            // can insert agents up to capacity
            int numAgentsToAdd = Math.min( this.problemSpaceAgentCapacity-agentsInProblem , agentQueue.size() );
            // if the queue exceeds capacity after removing every agent that can be added to the space at this time step
            // we return null (fail)
            if (this.agentQueue.size() - numAgentsToAdd > this.agentQueueCapacity) {
                this.breachedCapacity = true;
                wrapUp(parameters, null, timestepWithNewAgents);
                return null;
            }
            // add agents to the problem
            newArrivals.clear();
            for (int i = 0; i < numAgentsToAdd; i++) {
                newArrivals.add(this.agentQueue.remove());
            }
            // get a solution for the agents to follow as of this timestep
            Solution solutionAtTime = onlineSolver.newArrivals(timestepWithNewAgents, newArrivals);
            if(solutionAtTime == null){
                wrapUp(parameters, null, 0);
                return null; //probably as timeout
            }
            // agents that waited in queue should have an amount of stay moves at the start of their plans, equal
            // to the number of time steps that they waited for.
            appendStayMovesToStartOfPlans(solutionAtTime, newArrivals, instance);
            // store the solution
            solutionsAtTimes.put(timestepWithNewAgents, solutionAtTime);
//            // if space will become available before the next group of new agents, we want to give the solver a chance
//            // to add the agents in the queue. add an empty set of new agents at the end time of the first agent's plan
            int closestPlanEndTime = closestPlanEndTime(solutionAtTime, timestepWithNewAgents);
            agentsForTimes.remove(agentsForTimes.firstKey());
            if (! agentsForTimes.isEmpty() && ! this.agentQueue.isEmpty() && closestPlanEndTime < agentsForTimes.firstKey()){
                agentsForTimes.put(closestPlanEndTime, new ArrayList<>(0));
            }
            // if there will be no more new arriving agents, still need to empty the queue
            if(agentsForTimes.isEmpty() && ! this.agentQueue.isEmpty()) {
                agentsForTimes.put(closestPlanEndTime, new ArrayList<>(0));
            }
        }

        OnlineSolution solution = new OnlineSolution(solutionsAtTimes);

        //clear the solver and write the report
        wrapUp(parameters, solution, 0);

        // combine the stored solutions at times into a single online solution
        return solution;
    }

    private int agentsInProblem(int timestepWithNewAgents, SortedMap<Integer, Solution> solutionsAtTimes) {
        if (solutionsAtTimes.isEmpty()) return 0;
        else {
            int numAgents = 0;
            for (SingleAgentPlan plan : solutionsAtTimes.get(solutionsAtTimes.lastKey())){
                if (plan.moveAt(timestepWithNewAgents) != null) numAgents++;
            }
            return numAgents;
        }
    }

    private Integer closestPlanEndTime(Solution solutionAtTime, int timeNow) {
        int minEndTime = Integer.MAX_VALUE;
        for (SingleAgentPlan plan : solutionAtTime){
            if(plan.getEndTime() > timeNow){
                minEndTime = Math.min(minEndTime, plan.getEndTime());
            }
        }
        return minEndTime;
    }

    /**
     * Agents that waited in queue should have an amount of stay moves at the start of their plans, equal to the number
     * of times steps that they waited for. Creates new plans with those moves, replaces old plans with the new ones.
     * @param solutionAtTime the newly found solution for the given agents
     * @param newArrivals the agents that joined the
     * @param instance
     */
    private void appendStayMovesToStartOfPlans(Solution solutionAtTime, List<OnlineAgent> newArrivals, MAPF_Instance instance) {
        for (OnlineAgent agent : newArrivals){
            SingleAgentPlan oldPlan = solutionAtTime.getPlanFor(agent);
            SingleAgentPlan updatedPlan = new SingleAgentPlan(agent);
            int numStayMoves = oldPlan.getPlanStartTime() - agent.arrivalTime;
            I_Location garage = agent.getPrivateGarage(instance.map.getMapCell(agent.source));
            for (int i = 1; i <= numStayMoves; i++) {
                updatedPlan.addMove(new Move(agent, agent.arrivalTime + i, garage, garage));
            }
            for (Move move : oldPlan){
                updatedPlan.addMove(move);
            }
            solutionAtTime.putPlan(updatedPlan);
        }
    }


    @Override
    public String name() {
        return onlineSolver.name();
    }

    private void verifyAgentsUniqueId(List<Agent> agents) {
        HashSet<Integer> ids = new HashSet<>(agents.size());
        for (Agent agent :
                agents) {
            if(ids.contains(agent.iD)){
                throw new IllegalArgumentException("OnlineSolverContainer: Online solvers require all agents to have unique IDs");
            }
            else ids.add(agent.iD);
        }
    }

    /**
     * Groups agent by their arrival times. Converts any non-online {@link Agent agents} into {@link OnlineAgent online agents}
     * with the default arrival time.
     * @param agents agents to group
     * @return
     */
    public static SortedMap<Integer, List<OnlineAgent>> getAgentsByTime(List<? extends Agent> agents) {
        SortedMap<Integer, List<OnlineAgent>> result = new TreeMap<>(); //tree map to preserve order of arrival times
        ArrayList<OnlineAgent> onlineAgents = offlineToOnlineAgents(agents);

        //sort by time. Should already be sorted, so just in case.
        onlineAgents.sort(Comparator.comparing(OnlineAgent::getArrivalTime));

        //group by time
        for (int i = 0; i < onlineAgents.size();) {
            int currentTime = onlineAgents.get(i).arrivalTime;
            //find range with same arrival time
            int j = i;
            while(j < onlineAgents.size() && onlineAgents.get(j).arrivalTime == currentTime){
                j++;
            }
            //so the range we found is [i,j)

            result.put(currentTime, onlineAgents.subList(i, j /*end index is non-inclusive*/ ));

            i=j; //next group
        }

        return result;
    }


    /**
     * Cast agents to online agents. If they are regular Agents, create new OnlineAgents out of them with the default arrival time.
     * @param agents
     */
    private static ArrayList<OnlineAgent> offlineToOnlineAgents(List<? extends Agent> agents) {
        ArrayList<OnlineAgent> onlineAgents = new ArrayList<>(agents.size());
        for (Agent a :
                agents) {
            onlineAgents.add(a instanceof OnlineAgent ? (OnlineAgent)a : new OnlineAgent(a, ARRIVAL_TIME_FOR_OFFLINE_AGENTS));
        }
        return onlineAgents;
    }

    private void wrapUp(RunParameters parameters, OnlineSolution solution, int timestepWithNewAgents) {
        onlineSolver.writeReportAndClearData(solution);
        parameters.instanceReport.putStringValue(InstanceReport.StandardFields.solutionCostFunction, "SOC");
        parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.COR, costOfReroute);

        if(solution != null){
            parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.solutionCost, solution.sumIndividualCosts());
            parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.totalReroutesCost, solution.costOfReroutes(costOfReroute));
            parameters.instanceReport.putStringValue(InstanceReport.StandardFields.solution, solution.readableToString());
            parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.solved, 1);
            parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.capacityBreached, 0);
        }
        else{
            parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.solved, 0);
            if(this.breachedCapacity){
                parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.capacityBreached, 1);
                parameters.instanceReport.putIntegerValue(InstanceReport.StandardFields.throughput, timestepWithNewAgents);
            }
        }
    }

}
