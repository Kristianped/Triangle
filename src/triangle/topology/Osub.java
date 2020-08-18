package triangle.topology;

public class Osub {

    protected SubSegment seg;
    protected int orient; // Ranges from 0 to 1.

    public SubSegment getSegment() {
        return seg;
    }

    @Override
    public String toString() {
        if (seg == null)
            return "O-TID [null]";

        return String.format("O-TID [0}", seg.hash);
    }

    /**
     * Reverse the orientation of a subsegment [sym(ab) -> ba]
     */
    public void sym(Osub os) {
        os.seg = seg;
        os.orient = 1 - orient;
    }

    /**
     * Reverse the orientation of a subsegment [sym(ab) -> ba]
     */
    public void sym() {
        orient = 1 - orient;
    }

    /**
     * Find adjoining subsegment with the same origin [pivot(ab) -> a*]
     * Note: pivot() 
     * @param os
     */
    public void pivot(Osub os) {
        os = seg.subsegs[orient];
    }
}