package BasicCBS.Solvers.ConstraintsAndConflicts.Constraint;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.Maps.I_Location;
import BasicCBS.Solvers.Move;
import java.util.Objects;

public class RangeConstraint extends Constraint {

    int upperBound;
    int lowerBound;

    // depracated
    public RangeConstraint(Agent agent, int time, int range, I_Location prevLocation, I_Location location) {
        super(agent, time, prevLocation, location);
        this.setBounds(range);
    }


    public RangeConstraint(Agent agent, int time, int range, I_Location location) {
        super(agent, time, location);
        this.setBounds(range);
    }

    private void setBounds(int range){
        this.upperBound = this.time;
//        this.upperBound = this.time + ((RobustAgent)this.agent).k;
//        this.lowerBound = this.time;
        this.lowerBound = Math.max(0,this.time - range) ;
    }

    public boolean accepts(Move move){
        if(move == null) throw new IllegalArgumentException();
        if( move.prevLocation.equals(this.prevLocation) && this.time == 1){
            return false;
        }
//        boolean intersectsWithPrev = this.location.intersectsWith(move.prevLocation);
        boolean intersectsWithCur = move.prevLocation.equals(this.prevLocation);
        boolean timeAgent = this.inRange(move.timeNow) && this.agent.equals(move.agent);
        return !((intersectsWithCur) && timeAgent);
    }


    public boolean inRange(int time){
        return this.lowerBound <= time && time <= this.upperBound;
    }

    public Constraint getConstraint(int time){
//        return new Constraint_Robust(this.agent, time, this.prevLocation, this.location);
        return new Constraint(this.agent, time, this.prevLocation, this.location);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof RangeConstraint)) return false;
        if (!super.equals(o)) return false;
        RangeConstraint that = (RangeConstraint) o;
        return upperBound == that.upperBound &&
                lowerBound == that.lowerBound;
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), upperBound, lowerBound);
    }
}
