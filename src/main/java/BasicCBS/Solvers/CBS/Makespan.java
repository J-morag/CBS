package BasicCBS.Solvers.CBS;

import BasicCBS.Solvers.SingleAgentPlan;
import BasicCBS.Solvers.Solution;

public class Makespan implements CBS_Solver.CBSCostFunction {
    @Override
    public float solutionCost(Solution solution, CBS_Solver cbs) {
        int cost = 0;
        for (SingleAgentPlan plan : solution){
            cost = Math.max(plan.getCost(), cost);
        }
        return cost;
    }
}
