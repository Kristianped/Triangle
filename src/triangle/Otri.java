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

        return String.format("O-TID %d", tri.hash);
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

    public Otri shallowCopy() {
        Otri o = new Otri();
        copy(o);

        return o;
    }

    /// <summary>
    /// Find the abutting triangle; same edge. [sym(abc) -> ba*]
    /// </summary>
    /// Note that the edge direction is necessarily reversed, because the handle specified
    /// by an oriented triangle is directed counterclockwise around the triangle.
    /// </remarks>
    public void sym(Otri ot)
    {
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;
    }

    /// <summary>
    /// Find the abutting triangle; same edge. [sym(abc) -> ba*]
    /// </summary>
    public void sym()
    {
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge (counterclockwise) of a triangle. [lnext(abc) -> bca]
    /// </summary>
    public void lnext(Otri ot)
    {
        ot.tri = tri;
        ot.orient = plus1Mod3[orient];
    }

    /// <summary>
    /// Find the next edge (counterclockwise) of a triangle. [lnext(abc) -> bca]
    /// </summary>
    public void lnext()
    {
        orient = plus1Mod3[orient];
    }

    /// <summary>
    /// Find the previous edge (clockwise) of a triangle. [lprev(abc) -> cab]
    /// </summary>
    public void lprev(Otri ot)
    {
        ot.tri = tri;
        ot.orient = minus1Mod3[orient];
    }

    /// <summary>
    /// Find the previous edge (clockwise) of a triangle. [lprev(abc) -> cab]
    /// </summary>
    public void lprev()
    {
        orient = minus1Mod3[orient];
    }

    /// <summary>
    /// Find the next edge counterclockwise with the same origin. [onext(abc) -> ac*]
    /// </summary>
    public void onext(Otri ot)
    {
        //Lprev(ref ot);
        ot.tri = tri;
        ot.orient = minus1Mod3[orient];

        //ot.SymSelf();
        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge counterclockwise with the same origin. [onext(abc) -> ac*]
    /// </summary>
    public void onext()
    {
        //LprevSelf();
        orient = minus1Mod3[orient];

        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge clockwise with the same origin. [oprev(abc) -> a*b]
    /// </summary>
    public void oprev(Otri ot)
    {
        //Sym(ref ot);
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        //ot.LnextSelf();
        ot.orient = plus1Mod3[ot.orient];
    }

    /// <summary>
    /// Find the next edge clockwise with the same origin. [oprev(abc) -> a*b]
    /// </summary>
    public void oprev()
    {
        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        //LnextSelf();
        orient = plus1Mod3[orient];
    }

    /// <summary>
    /// Find the next edge counterclockwise with the same destination. [dnext(abc) -> *ba]
    /// </summary>
    public void dnext(Otri ot)
    {
        //Sym(ref ot);
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        //ot.LprevSelf();
        ot.orient = minus1Mod3[ot.orient];
    }

    /// <summary>
    /// Find the next edge counterclockwise with the same destination. [dnext(abc) -> *ba]
    /// </summary>
    public void dnext()
    {
        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        //LprevSelf();
        orient = minus1Mod3[orient];
    }

    /// <summary>
    /// Find the next edge clockwise with the same destination. [dprev(abc) -> cb*]
    /// </summary>
    public void dprev(Otri ot)
    {
        //Lnext(ref ot);
        ot.tri = tri;
        ot.orient = plus1Mod3[orient];

        //ot.SymSelf();
        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge clockwise with the same destination. [dprev(abc) -> cb*]
    /// </summary>
    public void dprev()
    {
        //LnextSelf();
        orient = plus1Mod3[orient];

        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge (counterclockwise) of the adjacent triangle. [rnext(abc) -> *a*]
    /// </summary>
    public void rnext(Otri ot)
    {
        //Sym(ref ot);
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        //ot.LnextSelf();
        ot.orient = plus1Mod3[ot.orient];

        //ot.SymSelf();
        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the next edge (counterclockwise) of the adjacent triangle. [rnext(abc) -> *a*]
    /// </summary>
    public void rnext()
    {
        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        //LnextSelf();
        orient = plus1Mod3[orient];

        //SymSelf();
        tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the previous edge (clockwise) of the adjacent triangle. [rprev(abc) -> b**]
    /// </summary>
    public void rprev(Otri ot)
    {
        //Sym(ref ot);
        ot.tri = tri.neighbors[orient].tri;
        ot.orient = tri.neighbors[orient].orient;

        //ot.LprevSelf();
        ot.orient = minus1Mod3[ot.orient];

        //ot.SymSelf();
        int tmp = ot.orient;
        ot.orient = ot.tri.neighbors[tmp].orient;
        ot.tri = ot.tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Find the previous edge (clockwise) of the adjacent triangle. [rprev(abc) -> b**]
    /// </summary>
    public void rprev()
    {
        //SymSelf();
        int tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;

        //LprevSelf();
        orient = minus1Mod3[orient];

        //SymSelf();
        tmp = orient;
        orient = tri.neighbors[tmp].orient;
        tri = tri.neighbors[tmp].tri;
    }

    /// <summary>
    /// Origin [org(abc) -> a]
    /// </summary>
    public Vertex org()
    {
        return tri.vertices[plus1Mod3[orient]];
    }

    /// <summary>
    /// Destination [dest(abc) -> b]
    /// </summary>
    public Vertex dest()
    {
        return tri.vertices[minus1Mod3[orient]];
    }

    /// <summary>
    /// Apex [apex(abc) -> c]
    /// </summary>
    public Vertex apex()
    {
        return tri.vertices[orient];
    }

    /// <summary>
    /// Copy an oriented triangle.
    /// </summary>
    public void copy(Otri ot)
    {
        ot.tri = tri;
        ot.orient = orient;
    }

    /// <summary>
    /// Test for equality of oriented triangles.
    /// </summary>
    public boolean equals(Otri ot)
    {
        return ((tri == ot.tri) && (orient == ot.orient));
    }


    /// <summary>
    /// Set Origin
    /// </summary>
    void setOrg(Vertex v)
    {
        tri.vertices[plus1Mod3[orient]] = v;
    }

    /// <summary>
    /// Set Destination
    /// </summary>
    void setDest(Vertex v)
    {
        tri.vertices[minus1Mod3[orient]] = v;
    }

    /// <summary>
    /// Set Apex
    /// </summary>
    void setApex(Vertex v)
    {
        tri.vertices[orient] = v;
    }

    /// <summary>
    /// Bond two triangles together at the resepective handles. [bond(abc, bad)]
    /// </summary>
    void bond(Otri ot)
    {
        tri.neighbors[orient].tri = ot.tri;
        tri.neighbors[orient].orient = ot.orient;

        ot.tri.neighbors[ot.orient].tri = this.tri;
        ot.tri.neighbors[ot.orient].orient = this.orient;
    }

    /// <summary>
    /// Dissolve a bond (from one side).
    /// </summary>
    /// <remarks>Note that the other triangle will still think it's connected to
    /// this triangle. Usually, however, the other triangle is being deleted
    /// entirely, or bonded to another triangle, so it doesn't matter.
    /// </remarks>
    void dissolve(Triangle dummy)
    {
        tri.neighbors[orient].tri = dummy;
        tri.neighbors[orient].orient = 0;
    }

    /// <summary>
    /// Infect a triangle with the virus.
    /// </summary>
    void infect()
    {
        tri.infected = true;
    }

    /// <summary>
    /// Cure a triangle from the virus.
    /// </summary>
    void uninfect()
    {
        tri.infected = false;
    }

    /// <summary>
    /// Test a triangle for viral infection.
    /// </summary>
    boolean isInfected()
    {
        return tri.infected;
    }

    /// <summary>
    /// Finds a subsegment abutting a triangle.
    /// </summary>
    Osub pivot()
    {
        return tri.subsegs[orient].shallowCopy();
    }

    /// <summary>
    /// Bond a triangle to a subsegment.
    /// </summary>
    void segBond(Osub os)
    {
        tri.subsegs[orient] = os.shallowCopy();
        os.seg.triangles[os.orient] = this.shallowCopy();
    }

    /// <summary>
    /// Dissolve a bond (from the triangle side).
    /// </summary>
    void segDissolve(SubSegment dummy)
    {
        tri.subsegs[orient].seg = dummy;
    }

    /// <summary>
    /// Check a triangle's deallocation.
    /// </summary>
    static boolean isDead(Triangle tria)
    {
        return tria.neighbors[0].tri == null;
    }

    /// <summary>
    /// Set a triangle's deallocation.
    /// </summary>
    static void kill(Triangle tri)
    {
        tri.neighbors[0].tri = null;
        tri.neighbors[2].tri = null;
    }
}
