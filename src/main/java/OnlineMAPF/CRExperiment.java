package OnlineMAPF;

import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.I_Solver;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.Solution;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Map;

/**
 * Experiment for competitive ratio experiments.
 * Adds the relevant fields to the report.
 */
public class CRExperiment extends OnlineExperiment {
    public CRExperiment(String experimentName, InstanceManager instanceManager, int[] coRs) {
        super(experimentName, instanceManager, coRs);
    }

    public CRExperiment(String experimentName, InstanceManager instanceManager, int numOfInstances, int[] coRs) {
        super(experimentName, instanceManager, numOfInstances, coRs);
    }

    @Override
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
            super.recordFailure(instance, minNumFailedAgentsForInstance, solver);
        }

        Integer elapsedTime = instanceReport.getIntegerValue(InstanceReport.StandardFields.elapsedTimeMS);
        if (elapsedTime != null) {
            System.out.println("Elapsed time (ms): " + elapsedTime);
        }

        if (!keepSolutionInReport) {
            instanceReport.putStringValue(InstanceReport.StandardFields.solution, "");
        }

        // add fields for this kind of experiment
        String[] fields = instance.extendedName.replace(".scen", "").split("_");
        instanceReport.putIntegerValue("graphDiameter", Integer.parseInt(fields[1].replace("D", "")));
//        instanceReport.putIntegerValue("TOA", Integer.parseInt(fields[2].replace("TOA", "")));
        instanceReport.putIntegerValue("TOA", solution.getPlanFor(instance.agents.get(1)).getPlanStartTime());
        instanceReport.putIntegerValue("delta1", Integer.parseInt(fields[3].replace("d", "")));
        instanceReport.putIntegerValue("delta2", Integer.parseInt(fields[4].replace("d", "")));
        instanceReport.putIntegerValue("a1PlanLength", solution.getPlanFor(instance.agents.get(0)).size()-1);
        instanceReport.putIntegerValue("a2PlanLength", solution.getPlanFor(instance.agents.get(1)).size()-1);


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
}
