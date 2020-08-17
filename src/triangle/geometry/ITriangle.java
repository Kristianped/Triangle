package triangle.geometry;

import triangle.topology.dcel.Vertex;

public interface ITriangle {

    // Gets or sets the triangle ID.
    public int getID();
    public void setID(int id);

    // Gets or sets a general purpose label.
    public int getLabel();
    public void setLabel(int label);

    // Gets or sets the triangle area constraint.
    public double getArea();
    public void setArea(double area);

    // Gets the vertex at a given index.
    public Vertex getVertex(int index);

    // Gets the ID of the vertex at a given index.
    public int getVertexID(int index);

    // Gets the neighbor triangle at given index.
    public ITriangle getNeighbor(int index);

    // Gets the ID of the neighbor triangle at given index.
    public int getNeighborID(int index);

    // Gets the segment at given index.
    public ISegment getSegment(int index);
}
