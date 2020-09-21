package OnlineMAPF;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.InstanceBuilders.InstanceBuilder_BGU;

public class OnlineInstanceBuilder_BGU extends InstanceBuilder_BGU {

    protected final int INDEX_AGENT_ARRIVAL_TIME = 5;

    @Override
    protected Agent agentFromStringArray(int dimensions, String[] agentLine) {
        Agent offlineAgent = super.agentFromStringArray(dimensions, agentLine);
        if(dimensions == 2){
            // will throw an IndexOutOfBoundsException if this instance isn't online
            return new OnlineAgent(offlineAgent,
                    Integer.parseInt(agentLine[INDEX_AGENT_ARRIVAL_TIME]) // add the agent's arrival time
            );
        }
        // nicetohave if(dimensions == 3)
        else return null;
    }
}