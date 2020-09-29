package triangle.io;

import triangle.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class TriangleWriter {

    public void exportMesh(Mesh mesh, String filename) throws IOException {
        String base = FileProcessor.removeExtension(filename);

        writePolygon(mesh, base.concat(".poly"));
        exportElements(mesh, base.concat(".ele"));
    }

    public void exportElements(Mesh mesh, String filename) throws IOException {
        Otri tri = new Otri();
        Vertex p1;
        Vertex p2;
        Vertex p3;
        boolean regions = mesh.getBehavior().isUseRegions();
        int j = 0;
        tri.setOrient(0);

        try (FileWriter fw = new FileWriter(filename); BufferedWriter writer = new BufferedWriter(fw)) {
            writer.write(String.format("%d %d %d", mesh.getTriangles().size(), 3, regions ? 1 : 0));
            writer.newLine();

            for (var item : mesh.getTriangles()) {
                tri.setTriangle(item);

                p1 = tri.org();
                p2 = tri.dest();
                p3 = tri.apex();

                // Triangle number, indices for three vertices.
                writer.write(String.format("%d %d %d %d", j, p1.getId(), p2.getId(), p3.getId()));

                if (regions)
                    writer.write(String.format(" %d", tri.getTriangle().getLabel()));

                writer.newLine();
                item.setID(j++);
            }
        }
    }

    public void exportPolygon(IPolygon polygon, String filename) {

    }

    private void writeNodes(Mesh mesh, String filename) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            writeNodes(writer, mesh);
        }
    }

    private void writeNodes(BufferedWriter writer, Mesh mesh) throws IOException {
        if (writer == null)
            throw new IOException("Writer to file not intialised");

        int outVertices = mesh.getVertices().size();
        int nextras = mesh.getAttributesPerVertex();

        Behavior behavior = mesh.getBehavior();

        if (behavior.isJettison())
            outVertices = mesh.getVertices().size() - mesh.getUndeads();

        // Number of vertices, number of dimensions, number of vertex attributes,
        // and number of boundary markers (zero or one).
        writer.write(String.format("%d %d %d %d", outVertices, mesh.getDimensions(), nextras, behavior.useBoundaryMarkers() ? 1 : 0));
        writer.newLine();

        if (mesh.getCurrentNumbering() == Enums.NodeNumbering.None)
            mesh.renumber();

        if (mesh.getCurrentNumbering() == Enums.NodeNumbering.Linear) {
            writeNodes(writer, mesh.getVertices(), behavior.useBoundaryMarkers(), nextras, behavior.isJettison());
        } else {
            // If numbering is not linear, a simple 'foreach' traversal of the dictionary
            // values doesn't reflect the actual numbering. Use an array instead.

            List<Vertex> nodes = new ArrayList<>(mesh.getVertices().size());

            for (var node : mesh.getVertices())
                nodes.set(node.getId(), node);

            writeNodes(writer, nodes, behavior.useBoundaryMarkers(), nextras, behavior.isJettison());
        }
    }

    private void writeNodes(BufferedWriter writer, Iterable<Vertex> nodes, boolean markers, int attribs, boolean jettison) throws IOException {
        int index = 0;

        for (var vertex : nodes) {
            if (!jettison || vertex.getType() != Enums.VertexType.UndeadVertex) {
                // Vertex number, x and y coordinates.
                writer.write(String.format("%d %4.3f %4.3f", index, vertex.getX(), vertex.getY()));


                // Write attributes.
                for (int i = 0; i < attribs; i++)
                    writer.write(String.format(" %4.3f", vertex.getAttributes()[i]));

                if (markers)
                    writer.write(String.format(" %d", vertex.getLabel()));

                writer.newLine();
                index++;
            }
        }
    }

    private void writePolygon(Mesh mesh, String filename) {
        writePolygon(mesh, filename, true);
    }

    private void writePolygon(Mesh mesh, String filename, boolean writeNodes) {

    }


}
