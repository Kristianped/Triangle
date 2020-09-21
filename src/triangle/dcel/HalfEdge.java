package triangle.dcel;

public class HalfEdge {

    protected int id;
    protected int mark;

    protected Vertex origin;
    protected Face face;
    protected HalfEdge twin;
    protected HalfEdge next;

    public HalfEdge(Vertex origin) {
        this.setOrigin(origin);
    }

    public HalfEdge(Vertex origin, Face face) {
        this.setOrigin(origin);
        this.setFace(face);

        if (face != null && face.getEdge() == null)
            face.setEdge(this);
    }

    @Override
    public String toString() {
        return String.format("HE-ID {0} (Origin = VID-{1}", getId(), getOrigin().getId());
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public int getBoundary() {
        return mark;
    }

    public void setBoundary(int mark) {
        this.mark = mark;
    }

    public Vertex getOrigin() {
        return origin;
    }

    public void setOrigin(Vertex origin) {
        this.origin = origin;
    }

    public Face getFace() {
        return face;
    }

    public void setFace(Face face) {
        this.face = face;
    }

    public HalfEdge getTwin() {
        return twin;
    }

    public void setTwin(HalfEdge twin) {
        this.twin = twin;
    }

    public HalfEdge getNext() {
        return next;
    }

    public void setNext(HalfEdge next) {
        this.next = next;
    }
}
