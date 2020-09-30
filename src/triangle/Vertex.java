package triangle;

public class Vertex extends Point {

    int hash;
    double[] attributes;
    Enums.VertexType type;
    Otri tri;

    /**
     * Initializes a new instance of the Vertex class
     */
    public Vertex() {
        this(0, 0);
    }

    /**
     * Initializes a new instance of the Vertex class
     * @param x - The x coordinate of the vertex
     * @param y - The y coordinate of the vertex
     */
    public Vertex(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Initializes a new instance of the Vertex class
     * @param x - The x coordinate of the vertex
     * @param y - The y coordinate of the vertex
     * @param mark - The boundary mark
     */
    public Vertex(double x, double y, int mark) {
        super(x, y, mark);
        this.type = Enums.VertexType.InputVertex;
        tri = new Otri();
    }

    /**
     * Initializes a new instance of the Vertex class
     * @param x - The x coordinate of the vertex
     * @param y - The y coordinate of the vertex
     * @param mark - The boundary mark
     * @param attribs - The number of point attributes
     */
    public Vertex(double x, double y, int mark, int attribs) {
        this(x, y, mark);

        if (attribs > 0)
            this.attributes = new double[attribs];
    }

    public double[] getAttributes() {
        return attributes;
    }

    public void setAttributes(double[] attributes) {
        this.attributes = attributes;
    }

    public Enums.VertexType getType() {
        return type;
    }

    public void setType(Enums.VertexType type) {
        this.type = type;
    }

    @Override
    public int hashCode() {
        return hash;
    }
}
