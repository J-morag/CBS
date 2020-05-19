package OnlineMAPF;

import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.InstanceProperties;
import Environment.A_RunManager;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;
import OnlineMAPF.Solvers.OnlineCBSSolver;
import OnlineMAPF.Solvers.OnlineSolverContainer;

import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class RunManagerOnlineRuntimes extends A_RunManager {

    String resultsOutputDir = IO_Manager.buildPath(new String[]{System.getProperty("user.home"), "CBS_Results"});

    /*  = Set Solvers =  */
    @Override
    protected void setSolvers() {
        OnlineCBSSolver preservingRootSolver = new OnlineCBSSolver(true);
        preservingRootSolver.name = "Preserving Root Online";
        OnlineCBSSolver naiveOnline = new OnlineCBSSolver(false);
        naiveOnline.name = "Naive Online";
        this.solvers.add(new OnlineSolverContainer(preservingRootSolver));
        this.solvers.add(new OnlineSolverContainer(naiveOnline));
    }

    /*  = Set Experiments =  */
    @Override
    protected void setExperiments() {
        addExperimentCompareRuntimes();
    }

    @Override
    public void runAllExperiments() {
        try {
            S_Metrics.setHeader(new String[]{
                    InstanceReport.StandardFields.experimentName,
                    InstanceReport.StandardFields.instanceName,
                    InstanceReport.StandardFields.mapName,
                    InstanceReport.StandardFields.agentSelection,
                    InstanceReport.StandardFields.arrivalDistribution,
                    InstanceReport.StandardFields.arrivalRate,
                    InstanceReport.StandardFields.numAgents,
                    InstanceReport.StandardFields.solver,
                    InstanceReport.StandardFields.solved,
                    InstanceReport.StandardFields.valid,
                    InstanceReport.StandardFields.elapsedTimeMS,
                    InstanceReport.StandardFields.solutionCost,
                    InstanceReport.StandardFields.numReroutes,
                    InstanceReport.StandardFields.COR,
                    InstanceReport.StandardFields.totalReroutesCost,
                    InstanceReport.StandardFields.numReroutes,
                    InstanceReport.StandardFields.solution});
        } catch (IOException e) {
            e.printStackTrace();
        }

        DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
        String pathWithStartTime = resultsOutputDir + "\\results " + dateFormat.format(System.currentTimeMillis()) + " .csv";
        try {
            S_Metrics.addOutputStream(new FileOutputStream((pathWithStartTime)));
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            S_Metrics.addOutputStream(System.out, S_Metrics::instanceReportToHumanReadableString);
        } catch (IOException e) {
            e.printStackTrace();
        }

        super.runAllExperiments();
        String pathWithEndTime = resultsOutputDir + "\\results " + dateFormat.format(System.currentTimeMillis()) + " .csv";

        try {
            S_Metrics.exportCSV(new FileOutputStream(pathWithEndTime)
//                    ,new String[]{   InstanceReport.StandardFields.experimentName,
//                            InstanceReport.StandardFields.experimentName,
//                            InstanceReport.StandardFields.agentSelection,
//                            InstanceReport.StandardFields.arrivalDistribution,
//                            InstanceReport.StandardFields.arrivalRate,
//                            InstanceReport.StandardFields.numAgents,
//                            InstanceReport.StandardFields.solver,
//                            InstanceReport.StandardFields.solved,
//                            InstanceReport.StandardFields.valid,
//                            InstanceReport.StandardFields.elapsedTimeMS,
//                            InstanceReport.StandardFields.solutionCost,
//                            InstanceReport.StandardFields.numReroutes,
//                            InstanceReport.StandardFields.COR,
//                            InstanceReport.StandardFields.totalReroutesCost,
//                            InstanceReport.StandardFields.numReroutes,
//                            InstanceReport.StandardFields.solution}
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
        S_Metrics.clearAll();
    }


    /* = Experiments =  */

    private void addExperimentCompareRuntimes() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\kiva"});

        /*  =   Set Properties   =  */
        int biggestNumAgents = 100;
        int[] agentAmounts = new int[biggestNumAgents];
        for (int i = 1; i <= biggestNumAgents; i++) {
            agentAmounts[i-1] = i;
        }
        InstanceProperties properties = new InstanceProperties(null, -1, agentAmounts);


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        OnlineExperiment experiment = new OnlineExperiment("Compare Run-times", instanceManager, new int[]{0, 20}, null, null);
        experiment.timeout = 5 * 60 * 1000;
        experiment.keepSolutionInReport = false;
        this.experiments.add(experiment);
    }
}
