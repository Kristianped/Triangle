package triangle.io;

import triangle.*;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Helper methods for reading Triangle file formats.
 */
public class TriangleReader {

    int startIndex = 0;

    /**
     * Reads geometry information from .node or .poly files
     * @param filename Filename
     * @return Polygon if read file is success, null if not
     * @throws IOException If an error occurs during reading of the file
     */
    public Polygon importPolygon(String filename) throws IOException {
        Polygon polygon = null;

        String base = FileProcessor.removeExtension(filename);
        String polyName = base.concat(".poly");
        String nodeName = base.concat(".node");

        if (Files.exists(Path.of(polyName))) {
            polygon = readPolygonFile(polyName);
        } else if (Files.exists(Path.of(nodeName))) {
            polygon = readNodeFile(nodeName);
        }

        return polygon;
    }

    /**
     * Read elements from an .ele file
     * @param filename
     * @return List of triangles read in from file
     * @throws IOException If an error occurs during reading of the file
     */
    public List<ITriangle> importTriangles(String filename) throws IOException {
        String base = FileProcessor.removeExtension(filename);
        String elementName = base.concat(".ele");

        if (Files.exists(Path.of(elementName)))
            return readElementFile(elementName);

        return null;
    }

    private Polygon readPolygonFile(String filename) throws IOException {
        Polygon data = null;
        startIndex = 0;
        String[] line;
        int inVertices = 0;
        int attributes = 0;
        int nodeMarkers = 0;

        try (FileReader fr = new FileReader(filename); BufferedReader br = new BufferedReader(fr)) {
            line = tryReadLine(br);

            if (line == null || line.length == 0)
                throw new IOException("Can't read input file");

            // Read number of vertices, number of dimensions, number of vertex
            // attributes, and number of boundary markers.
            inVertices = Integer.parseInt(line[0]);

            if (line.length > 1 && Integer.parseInt(line[1]) != 2)
                throw new IOException("Triangle only works with two-dimensional meshes");

            if (line.length > 2)
                attributes = Integer.parseInt(line[2]);

            if (line.length > 3)
                nodeMarkers = Integer.parseInt(line[3]);

            // Read the vertices
            if (inVertices > 0) {
                data = new Polygon(inVertices);

                for (int i = 0; i < inVertices; i++) {
                    line = tryReadLine(br);

                    if (line == null)
                        throw new IOException("Can't read input file (vertices)");

                    if (line.length < 3)
                        throw new IOException("Invalid vertex");

                    if (i == 0)
                        startIndex = Integer.parseInt(line[0]);

                    readVertex(data.getPoints(), line, attributes, nodeMarkers);
                }
            } else {
                // If the .poly file claims there are zero vertices, that means that
                // the vertices should be read from a separate .node file.
                data = readNodeFile(filename.replace(".poly", ".node"));
                inVertices = data.getPoints().size();
            }

            var points = data.getPoints();

            if (points.isEmpty())
                throw new IOException("No nodes available");

            int index = 0;

            for (Point p : points)
                p.setId(index++);

            // Read the segments from a .poly file.
            // Read number of segments and number of boundary markers.
            line = tryReadLine(br);

            if (br == null)
                throw new IOException("Can't read input file (segments)");

            int inSegments = Integer.parseInt(line[0]);
            int segmentMarkers = 0;

            if (line.length > 1)
                segmentMarkers = Integer.parseInt(line[1]);

            int end1;
            int end2;
            int mark;

            // Read and insert the segments.
            for (int i = 0; i < inSegments; i++) {
                line = tryReadLine(br);

                if (line == null)
                    throw new IOException("Can't read input file (segments)");

                if (line.length < 3)
                    throw new IOException("Segment has no endpoints");

                end1 = Integer.parseInt(line[1]) - startIndex;
                end2 = Integer.parseInt(line[2]) - startIndex;
                mark = 0;

                if (segmentMarkers > 0 && line.length > 3)
                    mark = Integer.parseInt(line[3]);

                if (end1 < 0 || end1 >= inVertices)
                    System.err.println("Invalid first endpoint of segment");
                else if (end2 < 0 || end2 >= inVertices)
                    System.err.println("Invalid second endpoint of segment");
                else
                    data.add(new Segment(points.get(end1), points.get(end2), mark));
            }

            // Read holes from a .poly file
            // Read the holes
            line = tryReadLine(br);

            if (line == null)
                throw new IOException("Can't read input file (holes)");

            int holes = Integer.parseInt(line[0]);

            if (holes > 0) {
                for (int i = 0; i < holes; i++) {
                    line = tryReadLine(br);

                    if (line == null)
                        throw new IOException("Can't read input file (holes)");

                    if (line.length < 3)
                        throw new IOException("Invalid hole");

                    data.getHoles().add(new Point(Double.parseDouble(line[1]), Double.parseDouble(line[2])));
                }
            }

            // Read area constrains (optional)
            line = tryReadLine(br);

            if (line != null) {
                int id;
                int regions = Integer.parseInt(line[0]);

                for (int i = 0; i < regions; i++) {
                    line = tryReadLine(br);

                    if (line == null)
                        throw new IOException("Can't read input file (regions)");

                    if (line.length < 4)
                        throw new IOException("Invalid region attributes");

                    if (tryParse(line[3]))
                        id = Integer.parseInt(line[3]);
                    else
                        id = i;

                    double area = 0;

                    if (line.length > 4 && tryParse(line[4]))
                        area = Double.parseDouble(line[4]);

                    // Triangle's .poly file format allows region definitions with
                    // either 4 or 5 parameters, and different interpretations for
                    // them depending on the number of parameters.
                    //
                    // See http://www.cs.cmu.edu/~quake/triangle.poly.html
                    //
                    // The Java version will interpret the fourth parameter always
                    // as an integer region id and the optional fifth parameter as
                    // an area constraint.

                    data.getRegions().add(new RegionPointer(
                            Double.parseDouble(line[1]),    // Region x
                            Double.parseDouble(line[2]),    // Region y
                            id,
                            area
                    ));
                }
            }
        }

        return data;
    }

    private Polygon readNodeFile(String filename) throws IOException {
        Polygon data = null;
        startIndex = 0;
        String[] line = null;
        int inVertices = 0;
        int attributes = 0;
        int nodeMarkers = 0;

        try (FileReader fr = new FileReader(filename); BufferedReader br = new BufferedReader(fr)) {
            line = tryReadLine(br);

            if (line == null)
                throw new IOException("Can't read input file");

            // Read number of vertices, number of dimensions, number of vertex
            // attributes, and number of boundary markers.
            inVertices = Integer.parseInt(line[0]);

            if (inVertices < 3)
                throw new IOException("Input must have at leat three input vertices");

            if (line.length > 1 && Integer.parseInt(line[1]) != 2)
                throw new IOException("Triangle only works with two-dimensional meshes");

            if (line.length > 2)
                attributes = Integer.parseInt(line[2]);

            if (line.length > 3)
                nodeMarkers = Integer.parseInt(line[3]);

            data = new Polygon(inVertices);

            // Read the vertices.
            for (int i = 0; i < inVertices; i++) {
                line = tryReadLine(br);

                if (line == null)
                    throw new IOException("Can't read input file (vertices)");

                if (line.length < 3)
                    throw new IOException("Invalid vertex");

                if (i == 0)
                    startIndex = Integer.parseInt(line[0]);

                readVertex(data.getPoints(), line, attributes, nodeMarkers);
            }
        }

        return data;
    }

    private List<ITriangle> readElementFile(String filename) throws IOException {
        List<ITriangle> triangles = null;
        int inTriangles = 0;
        int attributes = 0;
        boolean validRegion = false;

        try (FileReader fr = new FileReader(filename); BufferedReader br = new BufferedReader(fr)) {
            // Read number of elements and number of attribute;
            String[] line = tryReadLine(br);

            if (line == null)
                throw new IOException("Can't read input file (elements)");

            inTriangles = Integer.parseInt(line[0]);

            // We ignore index 1 (number of nodes per triangle)
            attributes = 0;

            if (line.length > 2) {
                attributes = Integer.parseInt(line[2]);
                validRegion = true;
            }

            if (attributes > 1)
                System.err.println("Triangle attributes not supported");

            triangles = new ArrayList<>(inTriangles);
            InputTriangle tri;

            // Read triangles
            for (int i = 0; i < inTriangles; i++) {
                line = tryReadLine(br);

                if (line == null)
                    throw new IOException("Can't read input file (elements)");

                if (line.length < 4)
                    throw new IOException("Triangle has no nodes");

                tri = new InputTriangle(
                        Integer.parseInt(line[1]) - startIndex,
                        Integer.parseInt(line[2]) - startIndex,
                        Integer.parseInt(line[3]) - startIndex);

                if (attributes > 0 && validRegion) {
                    if (tryParse(line[4]))
                        tri.setLabel(Integer.parseInt(line[4]));
                    else
                        tri.setLabel(0);
                }

                triangles.add(tri);
            }
        }


        return triangles;
    }

    private boolean tryParse(String s) {
        try {
            Double.parseDouble(s);
        } catch (Exception e) {
            return false;
        }

        return true;
    }

    private String[] tryReadLine(BufferedReader reader) throws IOException {
        if (!reader.ready())
            return null;

        String line = reader.readLine();

        while (isBlank(line) || line.startsWith("#")) {
            if (!reader.ready())
                return null;

            line = reader.readLine();
        }

        return line.trim().split("\\s+");

    }

    private boolean isBlank(CharSequence cs) {
        if (cs == null)
            return true;

        final int len = cs.length();

        if (len == 0)
            return true;

        for (int i = 0; i < len; i++)
            if (!Character.isWhitespace(cs.charAt(i)))
                return false;

        return true;
    }

    private void readVertex(List<Vertex> data, String[] line, int attributes, int marks) {
        double x = Double.parseDouble(line[1]);
        double y = Double.parseDouble(line[2]);

        var v = new Vertex(x, y);

        if (marks > 0 && line.length > 3 + attributes)
            v.setLabel(Integer.parseInt(line[3 + attributes]));

        if (attributes > 0) {
            var attribs = new double[attributes];

            for (int i = 0; i < attributes; i++)
                if (line.length > 3 + i)
                    attribs[i] = Double.parseDouble(line[3 + i]);

                v.setAttributes(attribs);
        }

        data.add(v);
    }
}
