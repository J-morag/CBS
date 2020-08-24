package OnlineMAPF.Solvers;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.InstanceBuilders.InstanceBuilder_BGU;
import BasicCBS.Instances.InstanceBuilders.InstanceBuilder_MovingAI;
import BasicCBS.Instances.InstanceManager;
import BasicCBS.Instances.InstanceProperties;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Instances.Maps.Coordinates.Coordinate_2D;
import BasicCBS.Instances.Maps.Coordinates.I_Coordinate;
import BasicCBS.Instances.Maps.Enum_MapCellType;
import BasicCBS.Instances.Maps.I_Map;
import BasicCBS.Instances.Maps.MapFactory;
import BasicCBS.Solvers.AStar.RunParameters_SAAStar;
import BasicCBS.Solvers.I_Solver;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.Solution;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.S_Metrics;
import OnlineMAPF.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.SortedMap;

import static org.junit.jupiter.api.Assertions.*;

class OnlineCBSSolverTest {

    private final Enum_MapCellType e = Enum_MapCellType.EMPTY;
    private final Enum_MapCellType w = Enum_MapCellType.WALL;
    private Enum_MapCellType[][] map_2D_circle = {
            {w, w, w, w, w, w},
            {w, w, e, e, e, w},
            {w, w, e, w, e, w},
            {w, w, e, e, e, w},
            {w, w, w, w, w, w},
            {w, w, w, w, w, w},
    };
    private I_Map mapCircle = MapFactory.newSimple4Connected2D_GraphMap(map_2D_circle);

    Enum_MapCellType[][] map_2D_empty = {
            {e, e, e, e, e, e},
            {e, e, e, e, e, e},
            {e, e, e, e, e, e},
            {e, e, e, e, e, e},
            {e, e, e, e, e, e},
            {e, e, e, e, e, e},
    };
    private I_Map mapEmpty = MapFactory.newSimple4Connected2D_GraphMap(map_2D_empty);

    Enum_MapCellType[][] map_2D_withPocket = {
            {e, w, e, w, e, w},
            {e, w, e, e, e, e},
            {w, w, e, w, w, e},
            {e, e, e, e, e, e},
            {e, e, w, e, w, w},
            {w, e, w, e, e, e},
    };
    private I_Map mapWithPocket = MapFactory.newSimple4Connected2D_GraphMap(map_2D_withPocket);

    Enum_MapCellType[][] map_2D_smallMaze = {
            {e, e, e, w, e, w},
            {e, w, e, e, e, e},
            {e, w, e, w, w, e},
            {e, e, e, e, e, e},
            {e, e, w, e, w, w},
            {w, w, w, e, e, e},
    };
    private I_Map mapSmallMaze = MapFactory.newSimple4Connected2D_GraphMap(map_2D_smallMaze);

    Enum_MapCellType[][] map_2D_corridors = {
            {w, w, e, e, e, w},
            {w, w, e, w, e, w},
            {w, w, e, w, e, w},
            {w, w, e, w, e, w},
            {w, w, e, w, e, w},
            {w, w, e, e, e, w},
    };
    private I_Map mapCorridors = MapFactory.newSimple4Connected2D_GraphMap(map_2D_corridors);

    private Enum_MapCellType[][] CompetitiveRatio_Diameter_5 = {
            {e, e, e, e},
            {e, w, w, e},
            {e, e, e, e},
    };
    private I_Map mapCompetitiveRatio_Diameter_5 = MapFactory.newSimple4Connected2D_GraphMap(CompetitiveRatio_Diameter_5);

    private I_Coordinate coor12 = new Coordinate_2D(1,2);
    private I_Coordinate coor13 = new Coordinate_2D(1,3);
    private I_Coordinate coor14 = new Coordinate_2D(1,4);
    private I_Coordinate coor22 = new Coordinate_2D(2,2);
    private I_Coordinate coor24 = new Coordinate_2D(2,4);
    private I_Coordinate coor32 = new Coordinate_2D(3,2);
    private I_Coordinate coor33 = new Coordinate_2D(3,3);
    private I_Coordinate coor34 = new Coordinate_2D(3,4);

    private I_Coordinate coor11 = new Coordinate_2D(1,1);
    private I_Coordinate coor43 = new Coordinate_2D(4,3);
    private I_Coordinate coor53 = new Coordinate_2D(5,3);
    private I_Coordinate coor54 = new Coordinate_2D(5,4);
    private I_Coordinate coor05 = new Coordinate_2D(0,5);

    private I_Coordinate coor04 = new Coordinate_2D(0,4);
    private I_Coordinate coor00 = new Coordinate_2D(0,0);
    private I_Coordinate coor01 = new Coordinate_2D(0,1);
    private I_Coordinate coor10 = new Coordinate_2D(1,0);

    private I_Coordinate coor02 = new Coordinate_2D(0, 2);

    private OnlineAgent agent33to12 = new OnlineAgent(0, coor33, coor12, 0);
    private OnlineAgent agent12to33 = new OnlineAgent(1, coor12, coor33, 0);
    private OnlineAgent agent53to05 = new OnlineAgent(2, coor53, coor05, 0);
    private OnlineAgent agent43to11 = new OnlineAgent(3, coor43, coor11, 0);
    private OnlineAgent agent04to54 = new OnlineAgent(4, coor04, coor54, 0);
    private OnlineAgent agent00to10 = new OnlineAgent(5, coor00, coor10, 0);
    private OnlineAgent agent10to00 = new OnlineAgent(6, coor10, coor00, 0);

    private OnlineAgent agent12to33t0 = new OnlineAgent(new Agent(1, coor12, coor33), 0);
    private OnlineAgent agent12to34t0 = new OnlineAgent(new Agent(2, coor12, coor33), 0);
    private OnlineAgent agent11to33t0 = new OnlineAgent(new Agent(3, coor12, coor33), 0);

    private OnlineAgent agent12to33t1 = new OnlineAgent(new Agent(4, coor12, coor33), 1);
    private OnlineAgent agent12to33t3 = new OnlineAgent(new Agent(5, coor12, coor33), 3);
    private OnlineAgent agent12to33t6 = new OnlineAgent(new Agent(6, coor12, coor33), 6);
    private OnlineAgent agent12to33t7 = new OnlineAgent(new Agent(7, coor12, coor33), 7);
    private OnlineAgent agent53to05t1 = new OnlineAgent(new Agent(8, coor53, coor05), 1);
    private OnlineAgent agent53to05t4 = new OnlineAgent(new Agent(9, coor53, coor05), 4);
    private OnlineAgent agent53to05t5 = new OnlineAgent(new Agent(10,  coor53, coor05), 5);
    private OnlineAgent agent53to05t6 = new OnlineAgent(new Agent(11, coor53, coor05), 6);
    private OnlineAgent agent53to05t7 = new OnlineAgent(new Agent(12, coor53, coor05), 7);
    private OnlineAgent agent12to33t0anotherOne = new OnlineAgent(new Agent(13, coor12, coor33), 0);

    private OnlineAgent agent13to10 = new OnlineAgent(0, coor13, coor10, 0);
    private OnlineAgent agent00to02 = new OnlineAgent(1, coor00, coor02, 3);

    InstanceBuilder_BGU builder = new OnlineInstanceBuilder_BGU();
    InstanceManager im_BGU = new InstanceManager(IO_Manager.buildPath( new String[]{   IO_Manager.testResources_Directory,"Instances", "Online"}),
            builder, new InstanceProperties());
    InstanceBuilder_MovingAI builderMovingAI = new OnlineInstanceBuilder_MovingAI();
    InstanceManager im_MovingAI = new InstanceManager(IO_Manager.buildPath( new String[]{   IO_Manager.testResources_Directory,"Instances", "Online", "MovingAI"}),
            builderMovingAI, new InstanceProperties(null, -1, new int[]{100}));

    private MAPF_Instance instanceEmpty1 = new MAPF_Instance("instanceEmpty", mapEmpty,
            new Agent[]{agent33to12, agent12to33, agent53to05, agent43to11, agent04to54, agent00to10, agent10to00});
    private MAPF_Instance instanceCircle1 = new MAPF_Instance("instanceCircle1", mapCircle, new Agent[]{agent33to12, agent12to33});
    private MAPF_Instance instanceCircle2 = new MAPF_Instance("instanceCircle1", mapCircle, new Agent[]{agent12to33, agent33to12});
    private MAPF_Instance instanceUnsolvable = new MAPF_Instance("instanceUnsolvable", mapWithPocket, new Agent[]{agent00to10, agent10to00});

    private MAPF_Instance instanceMultipleAgentsSameSource = new MAPF_Instance("instanceEmpty", mapEmpty, new Agent[]
            {agent12to33t0, agent12to34t0});
    private MAPF_Instance instanceMultipleAgentsSameTarget = new MAPF_Instance("instanceEmpty", mapEmpty, new Agent[]
            {agent12to33t0, agent11to33t0});
    private MAPF_Instance instanceMultipleAgentsSameSourcesTargets = new MAPF_Instance("instanceEmpty", mapEmpty, new Agent[]
            {agent12to33t0, agent12to34t0, agent11to33t0, agent12to33t1, agent12to33t3, agent12to33t6, agent12to33t7, agent53to05t1,
                    agent53to05t4, agent53to05t5, agent53to05t6, agent53to05t7, agent12to33t0anotherOne});
    private MAPF_Instance instanceDiameter5 = new MAPF_Instance("instanceDiameter5", mapCompetitiveRatio_Diameter_5,
            new Agent[]{agent13to10, agent00to02});

    private I_Solver solver = new OnlineSolverContainer(new OnlineCBSSolver());

    private InstanceReport instanceReport;

    @BeforeEach
    void setUp() {
        instanceReport = S_Metrics.newInstanceReport();
    }

    @AfterEach
    void tearDown() {
        S_Metrics.removeReport(instanceReport);
    }

    void validate(Solution solution, int numAgents, int optimalSOC, int optimalMakespan, MAPF_Instance instance){
        assertTrue(solution.solves(instance)); //is valid

        assertEquals(numAgents, solution.size()); // solution includes all agents
        assertEquals(optimalSOC, solution.sumIndividualCosts()); // SOC is optimal
        assertEquals(optimalMakespan, solution.makespan()); // makespan is optimal
    }

    @Test
    void emptyMapValidityTest1() {
        MAPF_Instance testInstance = instanceEmpty1;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        System.out.println(solved.readableToString());
        validate(solved, 7, solved.sumIndividualCosts(),solved.makespan(), testInstance); //need to find actual optimal costs
    }

    @Test
    void circleMapValidityTest1() {
        MAPF_Instance testInstance = instanceCircle1;
        Solution solved = solver.solve(testInstance, new RunParameters(System.currentTimeMillis() + (60*60*1000), null, instanceReport, null));

        System.out.println(solved.readableToString());
        validate(solved, 2, 8, 5, testInstance);

    }

    @Test
    void circleMapValidityTest2() {
        MAPF_Instance testInstance = instanceCircle2;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        System.out.println(solved.readableToString());
        validate(solved, 2, 8, 5, testInstance);
    }

    @Test
    void unsolvableIsNowSolvable() {
        MAPF_Instance testInstance = instanceUnsolvable;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        assertNotNull(solved);
        solved = new OnlineSolution(solved); // for the correct validation function
        validate(solved, 2, 4, 3, testInstance);
    }

    @Test
    void wasUnsolvableNowSolvableWithWaitBeforeEntering() {
        MAPF_Instance testInstance = instanceUnsolvable;

        // set start location to the agent's private garage
        RunParameters_SAAStar parameters = new RunParameters_SAAStar(instanceReport);
        OnlineAgent agent = ((OnlineAgent)testInstance.agents.get(0) );
        parameters.agentStartLocation = agent.getPrivateGarage(testInstance.map.getMapCell(agent.source));

        Solution solved = solver.solve(testInstance, parameters);

        assertNotNull(solved);
        System.out.println(solved.readableToString());
        // the latter agent (6) will stay at its garage and wait for the former agent (5) to get to its destination and disappear
        assertEquals(4, solved.sumIndividualCosts());
    }

    @Test
    void wasUnsolvableNowSolvableBecauseDisappearAtGoal() {
        OnlineAgent lateAgent10to00 = new OnlineAgent(agent10to00, 2);
        MAPF_Instance testInstance = new MAPF_Instance("nowSolvable", mapWithPocket, new Agent[]{agent00to10, lateAgent10to00});
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        assertNotNull(solved);
    }

    @Test
    void handlesMultipleAgentsSameSource() {
        MAPF_Instance testInstance = instanceMultipleAgentsSameSource;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        assertTrue(solved.solves(testInstance));
    }

    @Test
    void handlesMultipleAgentsSameTarget() {
        MAPF_Instance testInstance = instanceMultipleAgentsSameTarget;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        assertTrue(solved.solves(testInstance));
    }

    @Test
    void handlesMultipleAgentsSameSourcesTargets() {
        MAPF_Instance testInstance = instanceMultipleAgentsSameSourcesTargets;
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        assertTrue(solved.solves(testInstance));
    }

    @Test
    void isOnlySnapshotOptimalNotOptimal(){
        OnlineAgent agent53to02at0 = new OnlineAgent(0, coor53, coor02, 0);
        OnlineAgent agent12to53at4 = new OnlineAgent(1, coor12, coor53, 4);
        OnlineAgent agent22to53at4 = new OnlineAgent(2, coor22, coor53, 4);
        OnlineAgent[] agents = new OnlineAgent[3];
        agents[0] = agent53to02at0;
        agents[1] = agent12to53at4;
        agents[2] = agent22to53at4;
        MAPF_Instance testInstance = new MAPF_Instance("corridors snapshot optimal", mapCorridors, agents);
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS();
        InstanceReport tmpInstanceReport = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport));
        S_Metrics.removeReport(tmpInstanceReport);

        assertTrue(solved.solves(testInstance));

        /*
        The offline solver will send the agent arriving at t=0 through the right corridor, and the agents arriving at t=4 through
        the left corridor. The online solver will have already moved the agent arriving at t=0 a few moves to the left by
        the time the new agents arrive at t=4, so it will have them wait in their garages, resulting in a higher SOC.
         */
        int optimalCost = solvedOffline.sumIndividualCosts();
        int snapshotOptimalCost = solved.sumIndividualCosts();
        assertTrue(snapshotOptimalCost > optimalCost);
        assertEquals(18 , solved.sumIndividualCosts());
    }

    @Test
    void reroutesWhenAppropriate(){
        OnlineAgent agent53to02at0 = new OnlineAgent(0, coor53, coor02, 0);
        OnlineAgent agent12to53at2 = new OnlineAgent(1, coor12, coor53, 2);
        OnlineAgent agent22to53at2 = new OnlineAgent(2, coor22, coor53, 2);
        OnlineAgent[] agents = new OnlineAgent[3];
        agents[0] = agent53to02at0;
        agents[1] = agent12to53at2;
        agents[2] = agent22to53at2;
        MAPF_Instance testInstance = new MAPF_Instance("corridors should reroute", mapCorridors, agents);
        Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS();
        InstanceReport tmpInstanceReport = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport));
        S_Metrics.removeReport(tmpInstanceReport);

        assertTrue(solved.solves(testInstance));
        assertTrue(solvedOffline.solves(testInstance));

        /*
        The online solver will first send the initial agent through the left corridor, but when the new agents arrive,
        it wil reroute the initial agent through the right corridor, as that adds less to the cost than having both new
        agents wait or reroute.
        The offline solver will route the agent arriving at t=0 through the right from its first step, resulting in a
        lower SOC.
         */
        int optimalCost = solvedOffline.sumIndividualCosts();
        int snapshotOptimalCost = solved.sumIndividualCosts();
        assertTrue(snapshotOptimalCost > optimalCost);
        assertEquals(19 , solved.sumIndividualCosts());

        // check that the reroute happened rather than the less efficient option of having both new agents wait or go around.
        // these costs include the cost of mocing out of the garage, which is not included in SOC
        assertEquals(11, solved.getPlanFor(agent53to02at0).size());
        assertEquals(6, solved.getPlanFor(agent12to53at2).size());
        assertEquals(5, solved.getPlanFor(agent22to53at2).size());
    }

    @Test
    void biggerInstancesFromDiskBGU() {
        MAPF_Instance testInstance = null;
        while((testInstance = im_BGU.getNextInstance()) != null){
            System.out.println("------------ solving " + testInstance.name);
            Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

            assertTrue(solved.solves(testInstance));
            System.out.println(solved.readableToString());
        }
    }

    @Test
    void biggerInstancesFromDiskMovingAI() {
        MAPF_Instance testInstance = null;
        while((testInstance = im_MovingAI.getNextInstance()) != null){
            System.out.println("------------ solving " + testInstance.name);
            Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

            assertTrue(solved.solves(testInstance));
            System.out.println(solved.readableToString());
        }
    }

    /**
     * These instances returned an invalid (contains conflicts) solution when {@link OnlineSolution} was using a {@link java.util.Map}
     * as if it were a {@link SortedMap}. All improper uses of unsorted maps were replaced with sorted maps ({@link java.util.TreeMap}).
     */
    @Test
    void iteratesOverTimesInProperOrder(){
        MAPF_Instance testInstance = null;
        InstanceManager instanceManager = new InstanceManager(
                IO_Manager.buildPath( new String[]{   IO_Manager.testResources_Directory,"Instances", "Online", "MovingAI", "more_instances"}),
                builderMovingAI, new InstanceProperties(null, -1, new int[]{20}));
        while((testInstance = instanceManager.getNextInstance()) != null){
            System.out.println("------------ solving " + testInstance.name);
            Solution solved = solver.solve(testInstance, new RunParameters(instanceReport));

            assertTrue(solved.solves(testInstance));
            System.out.println(solved.readableToString());
        }
    }

    /*  = Cost Of Reroute =  */

    @Test
    void correctCostSmall() {
        OnlineAgent agent53to02at0 = new OnlineAgent(0, coor53, coor02, 0);
        OnlineAgent agent12to53at2 = new OnlineAgent(1, coor12, coor53, 2);
        OnlineAgent agent22to53at2 = new OnlineAgent(2, coor22, coor53, 2);
        OnlineAgent[] agents = new OnlineAgent[3];
        agents[0] = agent53to02at0;
        agents[1] = agent12to53at2;
        agents[2] = agent22to53at2;
        MAPF_Instance testInstance = new MAPF_Instance("corridors should reroute", mapCorridors, agents);

        RunParameters runParameters = new RunParameters(instanceReport);
        Solution solved = solver.solve(testInstance, runParameters);

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS();
        InstanceReport tmpInstanceReport = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport));
        S_Metrics.removeReport(tmpInstanceReport);

        assertTrue(solved.solves(testInstance));

        /*
        The online solver will first send the initial agent through the left corridor, but when the new agents arrive,
        it wil reroute the initial agent through the right corridor, as that adds less to the cost than having both new
        agents wait or reroute.
        The offline solver will route the agent arriving at t=0 through the right from its first step, resulting in a
        lower SOC.
         */
        int costOfReroute = 17;
        int costIncludingReroutes = ((OnlineSolution)solved).costOfReroutes(costOfReroute) + solved.sumIndividualCosts();
        // there was a single reroute, so cost should be SOC + the cost of a single reroute.
        assertEquals(19 + costOfReroute, costIncludingReroutes);
        // reroutes don't happen when solving offline, so should be 0.
        int rerouteCostsWhenOffline = ((OnlineSolution)solvedOffline).costOfReroutes(costOfReroute);
        assertEquals(0, rerouteCostsWhenOffline);
    }

    @Test
    void correctCostWaitingForGodot() {
        InstanceManager im = new InstanceManager(IO_Manager.buildPath( new String[]{   IO_Manager.testResources_Directory,"Instances", "Online", "MovingAI", "WaitingForGodot"}),
                builderMovingAI, new InstanceProperties(null, -1, new int[]{12}));
        MAPF_Instance testInstance = im.getNextInstance();
        System.out.println("------------ solving " + testInstance.name);
        RunParameters runParameters = new RunParameters(instanceReport);
        Solution solved = solver.solve(testInstance, runParameters);

        assertTrue(solved.solves(testInstance));
        System.out.println(solved.readableToString());

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS();
        InstanceReport tmpInstanceReport = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport));
        S_Metrics.removeReport(tmpInstanceReport);

        assertTrue(solved.solves(testInstance));

        int costOfReroute = 5;
        int costIncludingReroutes = ((OnlineSolution)solved).costOfReroutes(costOfReroute) + solved.sumIndividualCosts();
        // every time new agents arrive (every t > 1), agent 0 is rerouted (its plan of going down is replaced with a plan to wait 1 and then go down)
        // additionally, one more reroute may happen every time new agents arrive, if the solver chooses to prioritise the new agent over the existing agent
        assertEquals(( costIncludingReroutes - solved.sumIndividualCosts() - costOfReroute*4 ) % costOfReroute, 0 );
        // reroutes don't happen when solving offline, so should be 0.
        int rerouteCostsWhenOffline = ((OnlineSolution)solvedOffline).costOfReroutes(costOfReroute);
        assertEquals(0, rerouteCostsWhenOffline);
    }

    @Test
    void optimalWithCORSmall() {
        OnlineAgent agent53to02at0 = new OnlineAgent(0, coor53, coor02, 0);
        OnlineAgent agent12to53at2 = new OnlineAgent(1, coor12, coor53, 2);
        OnlineAgent agent22to53at2 = new OnlineAgent(2, coor22, coor53, 2);
        OnlineAgent[] agents = new OnlineAgent[3];
        agents[0] = agent53to02at0;
        agents[1] = agent12to53at2;
        agents[2] = agent22to53at2;
        MAPF_Instance testInstance = new MAPF_Instance("corridors should reroute", mapCorridors, agents);

        int costOfReroute = 17;
        RunParameters runParameters = new RunParametersOnline(instanceReport, costOfReroute);
        Solution solved = solver.solve(testInstance, runParameters);

        I_Solver offlineSolverGivenReroutes = new OnlineCompatibleOfflineCBS(null, -1, new COR_CBS_CostFunction(costOfReroute, null), new OnlineAStar(costOfReroute, null));
        InstanceReport tmpInstanceReport1 = S_Metrics.newInstanceReport();
        Solution solvedOfflineGivenReroutes = offlineSolverGivenReroutes.solve(testInstance, new RunParameters(tmpInstanceReport1));
        S_Metrics.removeReport(tmpInstanceReport1);

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS(null, -1, new COR_CBS_CostFunction(costOfReroute, null), new OnlineAStar(costOfReroute, null));
        InstanceReport tmpInstanceReport2 = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport2));
        S_Metrics.removeReport(tmpInstanceReport2);

        assertTrue(solved.solves(testInstance));

        System.out.println(solved.readableToString());

        /*
        The online solver now considers the cost of rerouting an agent (COR) vs the increase in SOC. It should now prefer
        to increase SOC at that time by preserving the plan of the initial agent, and having one of the new agents go around
        or wait for a cost increase of 5 over optimal individual plan, and having the other new agent wait for a cost increase
         of 4 over optimal individual plan.
         */
        int costIncludingReroutes = ((OnlineSolution)solved).costOfReroutes(costOfReroute) + solved.sumIndividualCosts();
        // no reroutes, and a higher SOC than without reroutes.
        // these costs include the cost of mocing out of the garage, which is not included in SOC
        int expectedAgent1Cost = 7;
        int expectedAgent2Cost = 10;
        int expectedAgent3Cost = 8;
        assertEquals(expectedAgent1Cost, solved.getPlanFor(agent53to02at0).size());
        assertEquals(expectedAgent2Cost, solved.getPlanFor(agent12to53at2).size());
        assertEquals(expectedAgent3Cost, solved.getPlanFor(agent22to53at2).size());
        assertEquals((expectedAgent1Cost-solved.size()) + expectedAgent2Cost + expectedAgent3Cost, costIncludingReroutes);
        // reroutes don't happen when solving offline, so should be 0.
        int rerouteCostsWhenOffline = ((OnlineSolution)solvedOffline).costOfReroutes(costOfReroute);
        assertEquals(0, rerouteCostsWhenOffline);
        // SOC should be unaffected when solving offline
        assertEquals(solvedOffline.sumIndividualCosts(), solvedOfflineGivenReroutes.sumIndividualCosts());
    }

    @Test
    void optimalWithCORWaitingForGodot() {
        InstanceManager im = new InstanceManager(IO_Manager.buildPath( new String[]{   IO_Manager.testResources_Directory,"Instances", "Online", "MovingAI", "WaitingForGodot"}),
                builderMovingAI, new InstanceProperties(null, -1, new int[]{12}));
        MAPF_Instance testInstance = im.getNextInstance();
        System.out.println("------------ solving " + testInstance.name);
        int costOfReroute = 5;
        RunParameters runParameters = new RunParametersOnline(instanceReport, costOfReroute);
        Solution solved = solver.solve(testInstance, runParameters);
        RunParameters runParameters2 = new RunParameters(instanceReport);
        Solution solvedWithoutCOR = solver.solve(testInstance, runParameters2);

        assertTrue(solved.solves(testInstance));
        System.out.println(solved.readableToString());

        I_Solver offlineSolver = new OnlineCompatibleOfflineCBS();
        InstanceReport tmpInstanceReport = S_Metrics.newInstanceReport();
        Solution solvedOffline = offlineSolver.solve(testInstance, new RunParameters(tmpInstanceReport));
        S_Metrics.removeReport(tmpInstanceReport);

        assertTrue(solved.solves(testInstance));

        /*
        Now that there is a cost to reroutes, the solver should prefer to not reroute agent 0, instead delaying the new
        agents. Every time new agents appear, the choice would again be made to prioritize maintaining the plans of the
        existing agents.
         */
        int costIncludingReroutes = ((OnlineSolution)solved).costOfReroutes(costOfReroute) + solved.sumIndividualCosts();
        assertEquals(solved.sumIndividualCosts(), costIncludingReroutes);
        // reroutes don't happen when solving offline, so should be 0.
        int rerouteCostsWhenOffline = ((OnlineSolution)solvedOffline).costOfReroutes(costOfReroute);
        assertEquals(0, rerouteCostsWhenOffline);
    }

    @Test
    void optimalWhenUsingPreserveSolutionInRoot() {
        MAPF_Instance testInstance = instanceDiameter5;
        I_Solver solverWithRootPreservation = new OnlineSolverContainer(new OnlineCBSSolver(true));
        Solution solved = solverWithRootPreservation.solve(testInstance, new RunParameters(instanceReport));

        System.out.println(solved.readableToString());
        validate(solved, 2, 9, 5, testInstance);
    }
}