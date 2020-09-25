package triangle;


public class Otri {

    static final int[] plus1Mod3 = { 1, 2, 0 };
    static final int[] minus1Mod3 = { 2, 0, 1 };

    Triangle tri;
    int orient;   // Ranges from 0 to 2

    public Triangle getTriangle() {
        return tri;
    }

    public void setTriangle(Triangle value) {
        tri = value;
    }
    public void setOrient(int value) {orient = value; }

    @Override
    public String toString() {
        if (tri == null)
            return "O-TID [null]";

        return String.format("O-TID {0}", tri.hash);
    }

    /*
     The following primitives are all described by Guibas and Stolfi.
     However, Guibas and Stolfi use an edge-based data structure,
     whereas I use a triangle-based data structure.

     lnext: finds the next edge (counterclockwise) of a triangle.

     onext: spins counterclockwise around a vertex; that is, it finds
     the next edge with the same origin in the counterclockwise direction. This
     edge is part of a different triangle.

     oprev: spins clockwise around a vertex; that is, it finds the
     next edge with the same origin in the clockwise direction.  This edge is
     part of a different triangle.

     dnext: spins counterclockwise around a vertex; that is, it finds
     the next edge with the same destination in the counterclockwise direction.
     This edge is part of a different triangle.

     dprev: spins clockwise around a vertex; that is, it finds the
     next edge with the same destination in the clockwise direction. This edge
     is part of a different triangle.

     rnext: moves one edge counterclockwise about the adjacent
     triangle. (It's best understood by reading Guibas and Stolfi. It
     involves changing triangles twice.)

     rprev: moves one edge clockwise about the adjacent triangle.
     (It's best understood by reading Guibas and Stolfi.  It involves
     changing triangles twice.)
    */

    /**
     * Find the abutting triangle; same edge. [sym(abc - ba*]
     * Note that the edge direction is necessarily reversed, because the handle specified
     * by an oriented triangle is directed counterclockwise around the triangle.
     */
    public void sym(Otri ot) {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;
    }

    /**
     * Find the abutting triangle; same edge. [sym(abc) - ba*]
     */
    public void sym() {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /**
     * Find the next edge (counterclockwise) of a triangle. [lnext(abc) - bca]
     */
    public void lnext(Otri ot) {
        ot.tri = tri;
        ot.orient = plus1Mod3[orient];
    }

    /**
     * Find the next edge (counterclockwise) of a triangle. [lnext(abc) - bca]
     */
    public void lnext() {
        orient = plus1Mod3[orient];
    }

    /**
     * Finds the prvious edge (clockwise) of a triangle. [lprev(abc) - cab]
     */
    public void lprev(Otri ot) {
        ot.tri = tri;
        ot.orient = minus1Mod3[orient];
    }

    /**
     * Finds the prvious edge (clockwise) of a triangle. [lprev(abc) - cab]
     */
    public void lprev() {
        orient = minus1Mod3[orient];
    }

    /**
     * Finds the next egde counterclockwise with the same origin. [onext(abc) - ac*]
     */
    public void onext(Otri ot) {
        ot.tri = tri;
        ot.orient = minus1Mod3[orient];

        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /**
     * Finds the next edge counterclockwise with the same origin. [onext(abc) - ac*]
     */
    public void onext() {
        orient = minus1Mod3[orient];

        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /**
     * Finds the next edge clockwise with the same origin. [oprev(abc) - a*b]
     */
    public void oprev(Otri ot) {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        ot.orient = plus1Mod3[ot.orient];
    }

    /**
     * Finds the next edge clockwise with the same origin. [oprev(abc) - a*b]
     */
    public void oprev() {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        orient = plus1Mod3[orient];
    }

    /**
     * Finds the next edge counterclockwise with the same destination. [dnext(abc) - *ba]
     */
    public void dnext(Otri ot) {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        ot.orient = minus1Mod3[ot.orient];
    }

    /**
     * Finds the next edge counterclockwise with the same destination. [dnext(abc) - *ba]
     */
    public void dnext() {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        orient = minus1Mod3[orient];
    }

    /**
     * Finds the next edge clockwise with the same destination. [dprev(abc) - cb*]
     */
    public void dprev(Otri ot) {
        ot.tri = tri;
        ot.orient = plus1Mod3[orient];

        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /**
     * Finds the next edge clockwise with the same destination. [dprev(abc) - cb*]
     */
    public void dprev() {
        orient = plus1Mod3[orient];

        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /**
     * Finds the next edge counterclockwise of the adjacent triangle. [rnext(abc) - *a*]
     */
    public void rnext(Otri ot) {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        ot.orient = plus1Mod3[ot.orient];

        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /**
     * Finds the next edge counterclockwise of the adjacent triangle. [rnext(abc) - *a*]
     */
    public void rnext() {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        orient = plus1Mod3[orient];

        tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /**
     * Finds the previous edge clockwise of the adjacent triangle. [rprev(abc) - b**]
     */
    public void rprev(Otri ot) {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        ot.orient = minus1Mod3[ot.orient];

        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /**
     * Finds the previous edge clockwise of the adjacent triangle. [rprev(abc) - b**]
     */
    public void rprev() {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        orient = minus1Mod3[orient];

        tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /**
     * Origin [org(abc) - a]
     */
    public Vertex org() {
        return tri.vertices[plus1Mod3[orient]];
    }

    /**
     * Destination [dest(abc) - b]
     */
    public Vertex dest() {
        return tri.vertices[minus1Mod3[orient]];
    }

    /**
     * Apex [apex(abc) - c]
     */
    public Vertex apex() {
        return tri.vertices[orient];
    }

    /**
     * Copy an oriented triangle.
     */
    public void copy(Otri ot) {
        ot.tri = tri;
        ot.orient = orient;
    }

    public boolean equals(Otri ot) {
        return ((tri == ot.tri) && (orient == ot.orient));
    }

    /**
     * Set origin.
     */
    void setOrg(Vertex v) {
        tri.vertices[plus1Mod3[orient]] = v;
    }

    /**
     * Set destination.
     */
    void setDest(Vertex v) {
        tri.vertices[minus1Mod3[orient]] = v;
    }

    /**
     * Set Apex.
     */
    void setApex(Vertex v) {
        tri.vertices[orient] = v;
    }

    /**
     * Bond two triangles together at the respective handles. [bond(abc, bad)]
     */
    void bond(Otri ot) {
        tri.neighbors[orient].tri = ot.tri;
        tri.neighbors[orient].orient = ot.orient;

        ot.tri.neighbors[ot.orient].tri = this.tri;
        ot.tri.neighbors[ot.orient].orient = this.orient;
    }

    /**
     * Dissolve a bond (from one side).
     * Note that the other triangle will still think it is connected to this triangle.
     * Ususlly, however, the other triangle is being deleted or bonded to another triangle,
     * so it doesn't really matter.
     */
    void dissolve(Triangle dummy) {
        tri.neighbors[orient].tri = dummy;
        tri.neighbors[orient].orient = 0;
    }

    /**
     * Infect a triangle with the virus.
     */
    void infect() {
        tri.infected = true;
    }

    /**
     * Cure a triangle from the virus.
     */
    void uninfect() {
        tri.infected = false;
    }

    /**
     * Tests a triangle for viral infection.
     */
    boolean isInfected() {
        return tri.infected;
    }

    /**
     * Finds a subsegment abutting a triangle
     */
    void pivot(Osub os) {
        os = tri.subsegs[orient];
    }

    /**
     * Bond a triangle to a subsegment
     */
    void segBond(Osub os) {
        tri.subsegs[orient] = os;
        os.seg.triangles[os.orient] = this;
    }

    /**
     * Dissolve a bond (from the triangle side)
     */
    void segDissolve(SubSegment dummy) {
        tri.subsegs[orient].seg = dummy;
    }

    /**
     * Check a triangle's deallocation
     */
    static boolean isDead(Triangle tria) {
        return tria.neighbors[0].tri == null;
    }

    /**
     * Set a triangle's deallocation
     */
    static void kill(Triangle tria) {
        tria.neighbors[0].tri = null;
        tria.neighbors[2].tri = null;
    }
}
