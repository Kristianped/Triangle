package triangle.topology;


import triangle.geometry.Vertex;

import java.util.concurrent.Flow;

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
     * Note: pivot() finds the other subsegment (from the same segment)
     * that shares the same origin
     */
    public void pivot(Osub os) {
        os = seg.subsegs[orient];
    }

    /**
     * Finds a triangle abutting a subsegment
     */
    public void pivot(Otri ot) {
        ot = seg.triangles[orient];
    }

    /**
     * Find next subsegment in sequence [next(ab) -> b*]
     */
    public void next(Osub os) {
        os = seg.subsegs[1 - orient];
    }

    /**
     * Find next subsegment in sequence [next(ab) -> b*]
     */
    public Osub next() {
        return seg.subsegs[1 - orient];
    }

    /**
     * Gets the origin of a subsegment
     */
    public Vertex org() {
        return seg.vertices[orient];
    }

    /**
     * Gets the destination of the subsegment
     */
    public Vertex dest() {
        return seg.vertices[1 - orient];
    }

    /**
     * Sets the origin or destination of the subsegment
     */
    protected void setOrg(Vertex vertex) {
        seg.vertices[orient] = vertex;
    }

    /**
     * Sets the destination of the subsegment
     */
    protected void setDest(Vertex vertex) {
        seg.vertices[1 - orient] = vertex;
    }

    /**
     * Gets the origin of the segment that includes the subsegment
     */
    protected Vertex segOrg() {
        return seg.vertices[2 + orient];
    }

    /**
     * Gets the destination of the segment that includes the subsegment
     */
    protected Vertex segDest() {
        return seg.vertices[3 - orient];
    }

    /**
     * Sets the origin of the segment that includes the subsegment
     */
    protected void setSegOrg(Vertex vertex) {
        seg.vertices[2 + orient] = vertex;
    }

    /**
     * Sets the destination of the segment that includes the subsegment
     */
    protected void setSegDest(Vertex vertex) {
        seg.vertices[3 - orient] = vertex;
    }

    /**
     * Bond two subsegments together [bond(abc), ba]
     */
    protected void bond(Osub os) {
        seg.subsegs[orient] = os;
        os.seg.subsegs[os.orient] = this;
    }

    /**
     * Dissolves a subsegment bod (from one side).
     * Note: The other subsegment will still think it's connected
     * to this subsegment
     * @param dummy
     */
    protected void dissolve(SubSegment dummy) {
        seg.subsegs[orient].seg = dummy;
    }

    /**
     * Test for equality
     */
    protected boolean equal(Osub os) {
        return ((seg == os.seg) && (orient == os.orient));
    }

    /**
     * Dissolve a bond (from the subsegment side)
     * @param dummy
     */
    protected void triDissolve(Triangle dummy) {
        seg.triangles[orient].tri = dummy;
    }

    /**
     * Check a subsegment's deallocation
     */
    protected static boolean isDead(SubSegment sub) {
        return sub.subsegs[0].seg == null;
    }

    /**
     * Sets a subsegment's deallocation
     */
    protected static void kill(SubSegment sub) {
        sub.subsegs[0].seg = null;
        sub.subsegs[1].seg = null;
    }
}