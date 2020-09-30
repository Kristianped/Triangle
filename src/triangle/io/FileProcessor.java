package triangle.io;

import triangle.*;
import triangle.meshing.Converter;
import triangle.meshing.IMesh;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class FileProcessor {

    public static final int NOT_FOUND = -1;

    public static final char EXTENSION_SEPARATOR = '.';

    public static final char UNIX_SEPARATOR = '/';

    public static final char WINDOWS_SEPARATOR = '\\';

    public static final char SYSTEM_SEPARATOR = File.separatorChar;

    public static boolean isSupported(String filename) {
        String extension = getExtension(filename);

        return extension.equals("poly") || extension.equals("node") || extension.equals("ele");
    }

    public static IMesh importMesh(String filename) throws IOException {
        if (isSupported(filename)) {
            TriangleReader reader = new TriangleReader();

            Polygon polygon = reader.importPolygon(filename);
            List<ITriangle> triangles = reader.importTriangles(filename);

            if (polygon != null && triangles != null && !triangles.isEmpty())
                return Converter.toMesh(polygon, triangles);
        }

        throw new IOException("Could not load '" + filename + "' file");
    }

    public static IPolygon importPolygon(String filename) throws IOException {
        return new TriangleReader().importPolygon(filename);
    }

    public static void exportMesh(IMesh mesh, String filename) throws IOException {
        new TriangleWriter().exportMesh((Mesh) mesh, filename);
    }

    public static void exportPolygon(IPolygon polygon, String filename) throws IOException {
        new TriangleWriter().exportPolygon(polygon, filename);
    }

    public static String removeExtension(final String filename) {
        if (filename == null)
            return null;

        final int index = indexOfExtension(filename);

        if (index == NOT_FOUND)
            return filename;
        else
            return filename.substring(0, index);
    }

    public static String getExtension(final String filename) {
        if (filename == null)
            return null;

        final int index = indexOfExtension(filename);

        if (index == NOT_FOUND)
            return "";
        else return filename.substring(index + 1);
    }

    private static int indexOfExtension(final String filename) {
        final int extensionPos = filename.lastIndexOf('.');
        final int lastSeparator = indexOfLastSeperator(filename);

        return lastSeparator > extensionPos ? NOT_FOUND : extensionPos;
    }

    private static int indexOfLastSeperator(final String filename) {
        final int lastUnixPos = filename.lastIndexOf(UNIX_SEPARATOR);
        final int lastWindowsPos = filename.lastIndexOf(WINDOWS_SEPARATOR);

        return Math.max(lastUnixPos, lastWindowsPos);
    }


}
