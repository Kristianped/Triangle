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
        return String.format("B-SID %d", subseg.seg.hash);
    }

    public Osub getSubseg() {
        return subseg;
    }

    public void setSubseg(Osub os) {
        subseg = os;
    }

    public Vertex getOrg() {
        return org;
    }

    public void setOrg(Vertex org) {
        this.org = org;
    }

    public Vertex getDest() {
        return dest;
    }

    public void setDest(Vertex dest) {
        this.dest = dest;
    }
}
