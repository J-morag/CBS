package BasicCBS.Solvers.ICTS.MergedMDDs;

import BasicCBS.Solvers.ICTS.MDDs.MDDNode;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Search node for searching the search space of a merged MDD (MDD from MDDs of multiple agents) without explicitly
 * creating the merged MDD.
 */
public class MergedMDDSpaceNode {
    private static final Comparator<MDDNode> mddNodesOrderComparator = Comparator.comparingInt(mddNode -> mddNode.getAgent().iD);
    private List<MDDNode> mddNodes;
    private MergedMDDSpaceNode parent;
    private int depth;
    public List<FatherSonMDDNodePair> generatingMoves;

    public MergedMDDSpaceNode(MergedMDDSpaceNode parent, int depth) {
        this.parent = parent;
        this.depth = depth;
    }

    public MergedMDDSpaceNode(MergedMDDSpaceNode parent, int depth, List<MDDNode> mddNodes) {
        this.parent = parent;
        this.depth = depth;
        this.setMDDNodes(mddNodes);
    }

    public MergedMDDSpaceNode() {
        this.parent = null;
        this.depth = 0;
    }

    public MergedMDDSpaceNode getParent() {
        return parent;
    }

    public void setMDDNodes(List<MDDNode> mddNodes){
        this.mddNodes = mddNodes;
        // we sort them so we can use them in more informed equals() and hashCode() functions.
        this.mddNodes.sort(mddNodesOrderComparator);
    }

    /**
     * Save the moves that generated this node. This is so we can delay the computationally expensive validation of the
     * combination of moves until we poll this node from open.
     * @param generatingMoves the moves that created this node from a previous node. can be null to remove (save heap space) after verifying this node.
     */
    public void setGeneratingMovesForLaterVerification(List<FatherSonMDDNodePair> generatingMoves){
        this.generatingMoves = generatingMoves;
    }

    public List<MDDNode> getMddNodes() {
        return mddNodes;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof MergedMDDSpaceNode)) return false;

        MergedMDDSpaceNode that = (MergedMDDSpaceNode) o;

        if (depth != that.depth) return false;
        return mddNodes.equals(that.mddNodes);
    }

    public int hashCode() {
        return mddNodes.hashCode();
    }

    public int getDepth() {
        return depth;
    }

    /**
     * Each list in this list is all moves that a single agent can make on their MDD from the mdd node that we have for
     * that agent in this current merged MDD node
     * @return
     */
    public List<List<FatherSonMDDNodePair>> getFatherSonPairsLists() {
        List<List<FatherSonMDDNodePair>> fatherSonPairs = new ArrayList<>(mddNodes.size());

        for (MDDNode father : mddNodes) {
            List<MDDNode> currentChildren = father.getNeighbors();
            List<FatherSonMDDNodePair> currentFatherSonPairs = new ArrayList<>(currentChildren.size());
            for (MDDNode children : currentChildren) {
                FatherSonMDDNodePair fatherSonMDDNodePair = new FatherSonMDDNodePair(father, children);
                currentFatherSonPairs.add(fatherSonMDDNodePair);
            }
            if(currentFatherSonPairs.isEmpty()){
                // we are at the deepest node of the MDD
                FatherSonMDDNodePair goalPair = new FatherSonMDDNodePair(father, father);
                currentFatherSonPairs.add(goalPair);
            }
            fatherSonPairs.add(currentFatherSonPairs);
        }

        return fatherSonPairs;
    }

    @Override
    public String toString() {
        return "MergedMDDNode{" +
                "mddNodes=" + mddNodes +
                ", depth=" + depth +
                '}';
    }

}
