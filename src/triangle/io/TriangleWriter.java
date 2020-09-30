package triangle.io;

import triangle.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TriangleWriter {

    public void exportMesh(Mesh mesh, String filename) throws IOException {
        String base = FileProcessor.removeExtension(filename);

        Locale.setDefault(new Locale("en", "US"));
        writePolygon(mesh, base.concat(".poly"));
        writeElements(mesh, base.concat(".ele"));
    }

    public void exportPolygon(IPolygon polygon, String filename) throws IOException {
        boolean hasMarkers = polygon.hasSegmentMarkers();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            writer.write(String.format("%d 2 0 %d", polygon.getPoints().size(), polygon.hasPointMarkers() ? 1 : 0));
            writer.newLine();

            // Write nodes to file
            writeNodes(writer, polygon.getPoints(), polygon.hasPointMarkers(), 0, false);

            // Number of segments, number of boundary markers (zero or one).
            writer.write(String.format("%d %d", polygon.getSegments().size(), polygon.hasPointMarkers() ? 1 : 0));
            writer.newLine();

            Vertex p, q;
            int j = 0;

            for (var seg : polygon.getSegments()) {
                p = seg.getVertex(0);
                q = seg.getVertex(1);

                // Segment number, indices of its two endpoints, and possibly a marker.
                if (hasMarkers)
                    writer.write(String.format("%d %d %d %d", j, p.getId(), q.getId(), seg.getLabel()));
                else
                    writer.write(String.format("%d %d %d", j, p.getId(), q.getId()));

                writer.newLine();
                j++;
            }

            // Holes
            j = 0;
            writer.write(String.format("%d", polygon.getHoles().size()));
            writer.newLine();

            for (var hole : polygon.getHoles()) {
                writer.write(String.format("%d %4.3f %4.3f", j++, hole.getX(), hole.getY()));
                writer.newLine();
            }

            // Regions
            if (polygon.getRegions().size() > 0) {
                j = 0;

                for (var region : polygon.getRegions()) {
                    writer.write(String.format("%d %4.3f %4.3f %d", j++, region.getPoint().getX(), region.getPoint().getY(), region.getId()));
                    writer.newLine();
                }
            }
        }
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
                nodes.add(node);

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

    private void writePolygon(Mesh mesh, String filename) throws IOException {
        writePolygon(mesh, filename, true);
    }

    private void writePolygon(Mesh mesh, String filename, boolean writeNodes) throws IOException {
        Osub subseg = new Osub();
        Vertex pt1, pt2;
        boolean useBoundaryMarkers = mesh.getBehavior().useBoundaryMarkers();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            if (writeNodes) {
                writeNodes(writer, mesh);
            } else {
                // The zero indicates that the vertices are in a separate .node file.
                // Followed by number of dimensions, number of vertex attributes,
                // and number of boundary markers (zero or one).
                writer.write(String.format("0 %d %d %d", mesh.getDimensions(), mesh.getNextras(), useBoundaryMarkers ? 1 : 0));
                writer.newLine();
            }

            // Number of segments, number of boundary markers (zero or one).
            writer.write(String.format("%d %d", mesh.getSegments().size(), useBoundaryMarkers ? 1 : 0));
            writer.newLine();

            subseg.setOrient(0);
            int j = 0;

            for (var item : mesh.getSegments()) {
                subseg.setSegment(item);

                pt1 = subseg.org();
                pt2 = subseg.dest();

                // Segment number, indices of its two endpoints, and possibly a marker.
                if (useBoundaryMarkers)
                    writer.write(String.format("%d %d %d %d", j, pt1.getId(), pt2.getId(), subseg.getSegment().getLabel()));
                else
                    writer.write(String.format("%d %d %d", j, pt1.getId(), pt2.getId()));

                writer.newLine();
                j++;
            }

            // Holes
            j = 0;
            writer.write(String.format("%d", mesh.getHoles().size()));
            writer.newLine();

            for (var hole : mesh.getHoles()) {
                writer.write(String.format("%d %4.3f %4.3f", j++, hole.getX(), hole.getY()));
                writer.newLine();
            }

            // Regions
            if (mesh.getRegions().size() > 0) {
                j = 0;
                writer.write(String.format("%d", mesh.getRegions().size()));
                writer.newLine();

                for (var region : mesh.getRegions()) {
                    writer.write(String.format("%d %4.3f %4.3f %d", j++, region.getPoint().getX(), region.getPoint().getY(), region.getId()));
                    writer.newLine();
                }
            }
        }
    }

    private void writeElements(Mesh mesh, String filename) throws IOException {
        Otri tri = new Otri();
        Vertex p1;
        Vertex p2;
        Vertex p3;
        boolean regions = mesh.getBehavior().useRegions();
        int j = 0;

        try (FileWriter fw = new FileWriter(filename); BufferedWriter writer = new BufferedWriter(fw)) {
            writer.write(String.format("%d %d %d", mesh.getTriangles().size(), 3, regions ? 1 : 0));
            writer.newLine();

            for (var item : mesh.getTriangles()) {
                tri.orient = 0;
                tri.tri = item;

                p1 = tri.org();
                p2 = tri.dest();
                p3 = tri.apex();

                // Triangle number, indices for three vertices.
                writer.write(String.format("%d %d %d %d", j, p1.getId(), p2.getId(), p3.getId()));

                if (regions)
                    writer.write(String.format(" %d", tri.tri.getLabel()));

                writer.newLine();
                item.setID(j++);
            }
        }
    }

}
