import org.junit.Test;
import static org.junit.Assert.assertNotNull;
import triangle.*;
import triangle.geometry.IPolygon;
import triangle.io.FileProcessor;
import triangle.meshing.ConstraintOptions;
import triangle.meshing.IMesh;
import triangle.meshing.QualityOptions;
import triangle.meshing.algorithm.SweepLine;

import java.io.IOException;

public class ReaderTest {

    @Test
    public void testImport() {
        try {
            var options = new ConstraintOptions();
            var quality = new QualityOptions();

            options.setConformingDelaunay(true);
            quality.setMinAngle(20);
            quality.setMaxAngle(100);

            // Dwyer
            IPolygon polygon = FileProcessor.importPolygon("A.poly");
            assertNotNull(polygon);

            IMesh mesh = PolygonHelper.triangulate(polygon, options, quality);
            assertNotNull(mesh);
            FileProcessor.exportMesh(mesh, "Export.poly");


            // Sweepline
            IPolygon polygon2 = FileProcessor.importPolygon("A.poly");
            assertNotNull(polygon);

            IMesh mesh2 = PolygonHelper.triangulate(polygon2, options, quality, new SweepLine());
            assertNotNull(mesh2);
            FileProcessor.exportMesh(mesh2, "Export2.poly");

        } catch (IOException e) {
            e.printStackTrace();
            assert false;
        }

        assert true;
    }
}
