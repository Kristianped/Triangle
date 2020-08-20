package triangle;

public class Enums {

    // Type of mesh vertex
    public enum VertexType { InputVertex, SegmentVertex, FreeVertex, DeadVertex, UndeadVertex };

    // Node renumbering algorithms
    public enum NodeNumbering { None, Linear, CuthillMcKee };

    // Labels that signify the result of point location
    public enum LocateResult { InTriangle, OnEdge, OnVertex, Outside };

    // Labels that signify the result of vertex insertion
    public enum InsertVertexResult { Successful, Encroaching, Violating, Duplicate };

    // Labels that signify the result of direction finding
    public enum FindDirectionResult { Within, Leftcollinear, Rightcollinear };
}
