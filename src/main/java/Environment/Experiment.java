package Environment;

import BasicCBS.Instances.InstanceBuilders.I_InstanceBuilder;
import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.MAPF_Instance;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;
import BasicCBS.Solvers.I_Solver;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.Solution;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Experiment class lets the user to specify the instances it needs for the experiment.
 * A_RunManager holds a list of Experiments, each Experiment receives Name and {@link InstanceManager}
 * {@link InstanceManager} receives a path to the instances' folder and a relevant parser {@link I_InstanceBuilder}
 * Optional, users can filter the instances in the folder by simply adding {@link BasicCBS.Instances.InstanceProperties} to {@link InstanceManager}
 * Make sure that {@link #setReport(MAPF_Instance, I_Solver)} is defined as you want
 */
public class Experiment {

    public final String experimentName;
    public final int numOfInstances;
    public int timeout = 300 * 1000;
    protected InstanceManager instanceManager;
    /**
     * When the experiment encounters an instance that was already tried with the same solver, and failed, and is now
     * being attempted with even more agents, that instance will be skipped.
     */
    public boolean skipAfterFail = true;

    /**
     * If set to false, the solution will be removed from reports before committing them. Solutions can be very large strings,
     * so removing them will save space in long experiments.
     */
    public boolean keepSolutionInReport = false;
    /**
     * If reports are written to an {@link java.io.OutputStream} (through {@link S_Metrics}) immediately upon being committed,
     * it may be preferred to just remove them afterwards, rather than keep accumulating them.
     */
    public boolean keepReportAfterCommit = true;
    /**
     * If set to true, will prompt {@link System} to collect garbage before every instance is attempted. This will reduce
     * average heap use (as it will be cleared often), and give every run a more similar starting point. The main thread
     * will sleep for 100 ms after calling for garbage collection, so that solver performance would not be affected by
     * garbage collection.
     */
    public boolean proactiveGarbageCollection = true;
    public int sleepTimeAfterGarbageCollection = 100;

    public Experiment(String experimentName, InstanceManager instanceManager) {
        this.experimentName = experimentName;
        this.instanceManager = instanceManager;
        this.numOfInstances = Integer.MAX_VALUE;
    }

    public Experiment(String experimentName, InstanceManager instanceManager, int numOfInstances) {
        this.experimentName = experimentName;
        this.instanceManager = instanceManager;
        this.numOfInstances = numOfInstances;
    }


    public InstanceReport setReport(MAPF_Instance instance, I_Solver solver) {
        InstanceReport instanceReport = S_Metrics.newInstanceReport();
        /*  = Put values in report =  */
        instanceReport.putStringValue(InstanceReport.StandardFields.experimentName, this.experimentName);
        instanceReport.putStringValue(InstanceReport.StandardFields.mapName, instance.name);
        instanceReport.putStringValue(InstanceReport.StandardFields.instanceName, instance.extendedName);
        instanceReport.putIntegerValue(InstanceReport.StandardFields.numAgents, instance.agents.size());
        instanceReport.putIntegerValue(InstanceReport.StandardFields.obstacleRate, instance.getObstaclePercentage());
        instanceReport.putStringValue(InstanceReport.StandardFields.solver, solver.name());

        return instanceReport;
    }


    public void runExperiment(I_Solver solver) {
        if (solver == null) {
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

            runInstanceOnSolver(solver, minNumFailedAgentsForInstance, instance);
        }
    }

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

            for (I_Solver solver :
                    solvers) {
                runInstanceOnSolver(solver, minNumFailedAgentsForInstance, instance);
            }
        }

    }

    protected void runInstanceOnSolver(I_Solver solver, Map<String, Integer> minNumFailedAgentsForInstance, MAPF_Instance instance) {
        if (proactiveGarbageCollection) {
            System.gc();
            try {
                Thread.sleep(sleepTimeAfterGarbageCollection);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // create report before skipping, so that output will be easier to read
        InstanceReport instanceReport = this.setReport(instance, solver);
        if (skipAfterFail && hasFailedWithLessAgents(instance, minNumFailedAgentsForInstance, solver)) {
            instanceReport.putIntegerValue(InstanceReport.StandardFields.skipped, 1);
            instanceReport.putIntegerValue(InstanceReport.StandardFields.solved, 0);
            return;
        }
        else{
            instanceReport.putIntegerValue(InstanceReport.StandardFields.skipped, 0);
        }

        RunParameters runParameters = getRunParameters(timeout, instanceReport);

        System.out.println("---------- solving " + instance.extendedName + " with " + instance.agents.size() + " agents ---------- with solver " + solver.name());
        System.out.println("Start time: " + new SimpleDateFormat("HH:mm:ss").format(System.currentTimeMillis()));

        Solution solution = solver.solve(instance, runParameters);

        System.out.println("Solved?: " + (solution != null ? "yes" : "no"));
        if (solution != null) {
            boolean validSolution = solution.solves(instance);
            System.out.println("Solution is " + (validSolution ? "valid" : "invalid!!!"));
            instanceReport.putIntegerValue(InstanceReport.StandardFields.valid, validSolution ? 1 : 0);
            System.out.println("Sum of Individual Costs: " + solution.sumIndividualCosts());
        } else { // failed to solve
            recordFailure(instance, minNumFailedAgentsForInstance, solver);
        }

        Integer elapsedTime = instanceReport.getIntegerValue(InstanceReport.StandardFields.elapsedTimeMS);
        if (elapsedTime != null) {
            System.out.println("Elapsed time (ms): " + elapsedTime);
        }

        if (!keepSolutionInReport) {
            instanceReport.putStringValue(InstanceReport.StandardFields.solution, "");
        }

        // Now that the report is complete, commit it
        try {
            instanceReport.commit();
            if (!keepReportAfterCommit) {
                S_Metrics.removeReport(instanceReport);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    protected RunParameters getRunParameters(int timeout, InstanceReport instanceReport){
        return new RunParameters(timeout, null, instanceReport, null);
    }

    protected void recordFailure(MAPF_Instance instance, Map<String, Integer> failedInstances, I_Solver solver) {
        Integer prevFailNumAgents = failedInstances.get(instanceAndSolverStringRepresentation(instance, solver));
        if (prevFailNumAgents == null || prevFailNumAgents > instance.agents.size()) {
            failedInstances.put(instanceAndSolverStringRepresentation(instance, solver), instance.agents.size());
        }
    }

    protected boolean hasFailedWithLessAgents(MAPF_Instance instance, Map<String, Integer> failedInstances, I_Solver solver) {
        Integer prevFailNumAgents = failedInstances.get(instanceAndSolverStringRepresentation(instance, solver));
        return prevFailNumAgents != null && prevFailNumAgents < instance.agents.size();
    }

    protected String instanceAndSolverStringRepresentation(MAPF_Instance instance, I_Solver solver) {
        return instance.extendedName + (solver != null ? solver.name() : "");
    }
}