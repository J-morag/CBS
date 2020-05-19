package OnlineMAPF;

import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.InstanceProperties;
import Environment.A_RunManager;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;
import OnlineMAPF.Solvers.OnlineAStar;
import OnlineMAPF.Solvers.OnlineCBSSolver;
import OnlineMAPF.Solvers.OnlineSolverContainer;
import OnlineMAPF.Solvers.ReplanSingle;

import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class RunManagerOnline extends A_RunManager {

    String resultsOutputDir = IO_Manager.buildPath(new String[]{System.getProperty("user.home"), "CBS_Results"});

    /*  = Set Solvers =  */
    @Override
    protected void setSolvers() {
        OnlineCBSSolver withCORSolver = new OnlineCBSSolver();
        withCORSolver.name = "COR solver";
        this.solvers.add(new OnlineSolverContainer(withCORSolver));
        OnlineCBSSolver withoutCORSolver = new OnlineCBSSolver();
        withoutCORSolver.name = "blind to COR";
        this.solvers.add(new OnlineSolverContainer(withoutCORSolver));
        withoutCORSolver.ignoreCOR = true;
        this.solvers.add(new ReplanSingle(new OnlineAStar()));
    }

    /*  = Set Experiments =  */
    @Override
    protected void setExperiments() {
//        addExperimentsUniqueAgents();
//        addExperimentRepeatingUniform();
//        addExperimentRepeatingNormal();
//        addExperimentsSmallMazes();
//        addExperimentsSmallCustom();
//        addExperimentWaitingForGodot();
        addExperimentExtensiveWithCOR();
//        addExperimentLongTime();
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

    private void addExperimentExtensiveWithCOR() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\extensive"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{60});

        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        OnlineExperiment experiment = new OnlineExperiment("Extensive With COR", instanceManager, new int[]{0, 1, 2, 3, 4, 50, 100}, null, null);
        experiment.keepSolutionInReport = false;
        this.experiments.add(experiment);
    }

    private void addExperimentsUniqueAgents() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\unique_agents_poisson"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{5, 10, 20, 40, 60, 80});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        this.experiments.add(new OnlineExperiment("unique_agents_poisson", instanceManager, null, null, null));
    }

    private void addExperimentRepeatingUniform() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\repeatingUniform_agents_poisson"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{5, 10, 20, 40, 60, 80});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        this.experiments.add(new OnlineExperiment("repeatingUniform_agents_poisson", instanceManager, null, null, null));
    }

    private void addExperimentRepeatingNormal() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\repeatingNormal_agents_poisson"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{5, 10, 20, 40, 60, 80});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        this.experiments.add(new OnlineExperiment("repeatingNormal_agents_poisson", instanceManager, null, null, null));
    }

    private void addExperimentsSmallMazes() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\small_mazes"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{5, 10, 20, 40, 60, 80, 100});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        this.experiments.add(new OnlineExperiment("smallMazes", instanceManager, null, null, null));
    }

    private void addExperimentsSmallCustom() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\small_custom"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{5, 10, 20, 40, 60, 80, 100});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        this.experiments.add(new OnlineExperiment("smallCustom", instanceManager, null, null, null));
    }


    private void addExperimentWaitingForGodot() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\waitingForGodot"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{21, 31, 41, 51, 61, 71});


        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        OnlineExperiment experiment = new OnlineExperiment("WaitingForGodot", instanceManager, null, null, null);
        experiment.keepSolutionInReport = true;
        this.experiments.add(experiment);
    }

    private void addExperimentCompareRuntimes() {
        /*  =   Set Path   =*/
        String path = IO_Manager.buildPath( new String[]{   IO_Manager.resources_Directory,
                "Instances\\\\Online\\\\MovingAI_Instances\\\\kiva"});

        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, new int[]{40, 50});

        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(path, new OnlineInstanceBuilder_MovingAI(), properties);

        /*  =   Add new experiment   =  */
        OnlineExperiment experiment = new OnlineExperiment("Compare Run-times", instanceManager, new int[]{0, 1, 2, 3, 10, 50, 100}, null, null);
        experiment.keepSolutionInReport = false;
        this.experiments.add(experiment);
    }

}
