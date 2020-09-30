package triangle.smoothing;

import triangle.Vertex;
import triangle.dcel.DcelVertex;
import triangle.dcel.Face;
import triangle.dcel.HalfEdge;
import triangle.voronoi.IVoronoiFactory;

import java.lang.reflect.Array;
import java.util.Arrays;

public class VoronoiFactory implements IVoronoiFactory {

    ObjectPool<DcelVertex> vertices;
    ObjectPool<HalfEdge> edges;
    ObjectPool<Face> faces;

    public VoronoiFactory() {
        vertices = new ObjectPool<>(DcelVertex.class);
        edges = new ObjectPool<>(HalfEdge.class);
        faces = new ObjectPool<>(Face.class);
    }

    @Override
    public void initialize(int vertexCount, int edgeCount, int faceCount) {
        vertices.setCapacity(vertexCount);
        edges.setCapacity(edgeCount);
        faces.setCapacity(faceCount);

        for (int i = vertices.getCount(); i < vertexCount; i++)
            vertices.put(new DcelVertex(0, 0));

        for (int i = edges.getCount(); i < edgeCount; i++)
            edges.put(new HalfEdge(null));

        for (int i = faces.getCount(); i < faceCount; i++)
            faces.put(new Face(null));

        reset();
    }

    @Override
    public void reset() {
        vertices.release();
        edges.release();
        faces.release();
    }

    @Override
    public DcelVertex createVertex(double x, double y) {
        DcelVertex vertex = vertices.tryGet();

        if (vertex != null) {
            vertex.setX(x);
            vertex.setY(y);
            vertex.setLeaving(null);
        } else {
            vertex = new DcelVertex(x, y);
            vertices.put(vertex);
        }

        return vertex;
    }

    @Override
    public HalfEdge createHalfEdge(DcelVertex origin, Face face) {
        HalfEdge edge = edges.tryGet();

        if (edge != null) {
            edge.setOrigin(origin);
            edge.setFace(face);
            edge.setNext(null);
            edge.setTwin(null);

            if (face != null && face.getEdge() == null)
                face.setEdge(edge);
        } else {
            edge = new HalfEdge(origin, face);
            edges.put(edge);
        }

        return edge;
    }

    @Override
    public Face createFace(Vertex vertex) {
        Face face = faces.tryGet();

        if (face != null) {
            face.setId(vertex.getId());
            face.setGenerator(vertex);
            face.setEdge(null);
        } else {
            face = new Face(vertex);
            faces.put(face);
        }

        return face;
    }
}

class ObjectPool<T extends Object> {

    int index;
    int count;

    T[] pool;

    public ObjectPool(Class<T> clazz) {
        this(clazz, 3);
    }

    public ObjectPool(Class<T> clazz, int capacity) {
        this.index = 0;
        this.count = 0;
        this.pool = (T[]) Array.newInstance(clazz, capacity);
    }

    public int getCount() {
        return count;
    }

    public int getCapacity() {
        return pool.length;
    }

    public void setCapacity(int value) {
        resize(value);
    }

    public T tryGet() {
        if (this.index < this.count)
            return this.pool[this.index++];

        return null;
    }

    public void put(T obj) {
        var capacity = this.pool.length;

        if (capacity <= this.count)
            resize(2 * capacity);

        this.pool[this.count++] = obj;
        this.index++;
    }

    public void release() {
        this.index = 0;
    }

    private void resize(int size) {
        if (size > this.count)
            pool = Arrays.copyOf(pool, size);
    }
}