package triangle;

/**
 * A queue used to store enroached subsegments.<br><br>
 * Note: Each subsegment's vertices are stored so that we can check whether a
 * subsegment is still the same
 */
public class BadSubSeg {

    Osub subseg;         // An enroached subsegment
    Vertex org, dest;    // Its two vertices

    @Override
    public int hashCode() {
        return subseg.seg.hash;
    }

    @Override
    public String toString() {
        return String.format("B-SID {0}", subseg.seg.hash);
    }
}
