package OnlineMAPF;

import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.I_Solver;
import BasicCBS.Solvers.RunParameters;
import Environment.Experiment;
import Environment.Metrics.InstanceReport;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public class OnlineExperiment extends Experiment {

    /**
     * An array of COR (Cost Of Reroute) to run each instance under.
     */
    public final int[] CORs;
    private int currentCOR = 0;
    public final int[] spaceCapacities;
    private int currentSpaceCapacity = 0;
    public final int[] queueCapacities;
    private int currentQueueCapacity = 0;

    /**
     * {@inheritDoc}
     * @param experimentName {@inheritDoc}
     * @param instanceManager {@inheritDoc}
     * @param coRs an array of COR (Cost Of Reroute) to run each instance under
     * @param spaceCapacities
     * @param queueCapacities
     */
    public OnlineExperiment(String experimentName, InstanceManager instanceManager, int[] coRs, int[] spaceCapacities, int[] queueCapacities) {
        super(experimentName, instanceManager);
        CORs = Objects.requireNonNullElse(coRs, new int[]{0});
        this.spaceCapacities = Objects.requireNonNullElse(spaceCapacities, new int[]{0});
        this.queueCapacities = Objects.requireNonNullElse(queueCapacities, new int[]{0});
    }

    /**
     * {@inheritDoc}
     * @param experimentName {@inheritDoc}
     * @param instanceManager {@inheritDoc}
     * @param numOfInstances {@inheritDoc}
     * @param coRs an array of COR (Cost Of Reroute) to run each instance under
     * @param spaceCapacities
     * @param queueCapacities
     */
    public OnlineExperiment(String experimentName, InstanceManager instanceManager, int numOfInstances, int[] coRs, int[] spaceCapacities, int[] queueCapacities) {
        super(experimentName, instanceManager, numOfInstances);
        CORs = Objects.requireNonNullElse(coRs, new int[]{0});
        this.spaceCapacities = Objects.requireNonNullElse(spaceCapacities, new int[]{0});
        this.queueCapacities = Objects.requireNonNullElse(queueCapacities, new int[]{0});
    }

    @Override
    public InstanceReport setReport(MAPF_Instance instance, I_Solver solver) {
        InstanceReport instanceReport = super.setReport(instance, solver);
        OnlineMAPF_Instance onlineMAPF_instance = (OnlineMAPF_Instance)instance;
        instanceReport.putStringValue(InstanceReport.StandardFields.agentSelection, onlineMAPF_instance.agentSelection);
        instanceReport.putStringValue(InstanceReport.StandardFields.arrivalDistribution, onlineMAPF_instance.arrivalDistribution);
        instanceReport.putStringValue(InstanceReport.StandardFields.arrivalRate, onlineMAPF_instance.arrivalRate);
        instanceReport.putIntegerValue(InstanceReport.StandardFields.COR, currentCOR);
        return instanceReport;
    }

    @Override
    public void runExperiment(List<I_Solver> solvers) {
        if (solvers == null) {
            return;
        }

        instanceManager.resetPathIndex();
        /*
         * Keeps a record of failed instances attempted by a solver, and the minimum number of agents attempted on that
         *  instance that produced a failure.
         */
        Map<String, Integer> minNumFailedAgentsForInstance = new HashMap<>();

        for (int i = 0; i < this.numOfInstances; i++) {

            MAPF_Instance instance = instanceManager.getNextInstance();

            if (instance == null) {
                break;
            }

            for (int cor : CORs) {
                currentCOR = cor;
                for(int spaceCapacity : spaceCapacities){
                    currentSpaceCapacity = spaceCapacity;
                    for (int queueCapacity : queueCapacities){
                        currentQueueCapacity = queueCapacity;
                        for (I_Solver solver :
                                solvers) {
                            runInstanceOnSolver(solver, minNumFailedAgentsForInstance, instance);
                        }
                    }
                }
            }

        }

    }

    @Override
    protected RunParameters getRunParameters(int timeout, InstanceReport instanceReport) {
        RunParametersOnline runParametersOnline = new RunParametersOnline(timeout, null, instanceReport, null, currentCOR);
        runParametersOnline.problemSpaceAgentCapacity = currentSpaceCapacity;
        runParametersOnline.agentQueueCapacity = currentQueueCapacity;
        return runParametersOnline;
    }
}
