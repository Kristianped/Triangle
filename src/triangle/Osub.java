package triangle;


public class Osub {

    SubSegment seg;
    int orient; // Ranges from 0 to 1.

    @Override
    public String toString() {
        if (seg == null)
            return "O-TID [null]";

        return String.format("O-TID %d", seg.hash);
    }

    public SubSegment getSegment() {
        return seg;
    }

    public Osub shallowCopy() {
        Osub os = new Osub();
        os.orient = orient;
        os.seg = seg;
        return os;
    }

    /// <summary>
    /// Reverse the orientation of a subsegment. [sym(ab) -> ba]
    /// </summary>
    public void sym(Osub os)
    {
        os.seg = seg;
        os.orient = 1 - orient;
    }

    /// <summary>
    /// Reverse the orientation of a subsegment. [sym(ab) -> ba]
    /// </summary>
    public void sym()
    {
        orient = 1 - orient;
    }

    /// <summary>
    /// Find adjoining subsegment with the same origin. [pivot(ab) -> a*]
    /// </summary>
    /// <remarks>spivot() finds the other subsegment (from the same segment)
    /// that shares the same origin.
    /// </remarks>
    public Osub pivotSub()
    {
        return seg.subsegs[orient].shallowCopy();
    }

    /// <summary>
    /// Finds a triangle abutting a subsegment.
    /// </summary>
    Otri pivotTri()
    {
        return seg.triangles[orient].shallowCopy();
    }

    /// <summary>
    /// Find next subsegment in sequence. [next(ab) -> b*]
    /// </summary>
    public Osub next()
    {
        return seg.subsegs[1 - orient].shallowCopy();
    }

    /// <summary>
    /// Get the origin of a subsegment
    /// </summary>
    public Vertex org()
    {
        return seg.vertices[orient];
    }

    /// <summary>
    /// Get the destination of a subsegment
    /// </summary>
    public Vertex dest()
    {
        return seg.vertices[1 - orient];
    }

    /// <summary>
    /// Set the origin or destination of a subsegment.
    /// </summary>
    void setOrg(Vertex vertex)
    {
        seg.vertices[orient] = vertex;
    }

    /// <summary>
    /// Set destination of a subsegment.
    /// </summary>
    void setDest(Vertex vertex)
    {
        seg.vertices[1 - orient] = vertex;
    }

    /// <summary>
    /// Get the origin of the segment that includes the subsegment.
    /// </summary>
    Vertex segOrg()
    {
        return seg.vertices[2 + orient];
    }

    /// <summary>
    /// Get the destination of the segment that includes the subsegment.
    /// </summary>
    Vertex segDest()
    {
        return seg.vertices[3 - orient];
    }

    /// <summary>
    /// Set the origin of the segment that includes the subsegment.
    /// </summary>
    void setSegOrg(Vertex vertex)
    {
        seg.vertices[2 + orient] = vertex;
    }

    /// <summary>
    /// Set the destination of the segment that includes the subsegment.
    /// </summary>
    void setSegDest(Vertex vertex)
    {
        seg.vertices[3 - orient] = vertex;
    }

    /// <summary>
    /// Bond two subsegments together. [bond(abc, ba)]
    /// </summary>
    void bond(Osub os)
    {
        seg.subsegs[orient] = os.shallowCopy();
        os.seg.subsegs[os.orient] = this.shallowCopy();
    }

    /// <summary>
    /// Dissolve a subsegment bond (from one side).
    /// </summary>
    /// <remarks>Note that the other subsegment will still think it's
    /// connected to this subsegment.</remarks>
    void dissolve(SubSegment dummy)
    {
        seg.subsegs[orient].seg = dummy;
    }

    /// <summary>
    /// Test for equality of subsegments.
    /// </summary>
    boolean equal(Osub os)
    {
        return ((seg == os.seg) && (orient == os.orient));
    }

    /// <summary>
    /// Dissolve a bond (from the subsegment side).
    /// </summary>
    void triDissolve(Triangle dummy)
    {
        seg.triangles[orient].tri = dummy;
    }

    /// <summary>
    /// Check a subsegment's deallocation.
    /// </summary>
    static boolean isDead(SubSegment sub)
    {
        return sub.subsegs[0].seg == null;
    }

    /// <summary>
    /// Set a subsegment's deallocation.
    /// </summary>
    static void kill(SubSegment sub)
    {
        sub.subsegs[0].seg = null;
        sub.subsegs[1].seg = null;
    }
}