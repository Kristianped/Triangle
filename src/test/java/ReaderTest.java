import org.junit.Test;
import static org.junit.Assert.assertNotNull;
import triangle.*;
import triangle.io.FileProcessor;

import java.io.IOException;

public class ReaderTest {

    @Test
    public void testImport() {
        try {
            IPolygon polygon = FileProcessor.importPolygon("A.poly");
            assertNotNull(polygon);

            var options = new ConstraintOptions();
            var quality = new QualityOptions();

            options.setConformingDelaunay(true);
            quality.setMinAngle(20);
            quality.setMaxAngle(100);

            //IMesh mesh = PolygonHelper.triangulate(polygon, options, quality);
            //assertNotNull(mesh);
            //FileProcessor.exportMesh(mesh, "Export.poly");

            IMesh mesh2 = PolygonHelper.triangulate(polygon, options, quality, new SweepLine());
            assertNotNull(mesh2);
            FileProcessor.exportMesh(mesh2, "Export2.poly");

            Statistic statistic = new Statistic();

            statistic.update((Mesh) mesh2, 10);
            quality.setMaxArea(0.25 * statistic.LargestArea());
            mesh2.refine(quality, true);

            statistic.update((Mesh) mesh2, 10);
            quality.setMaxArea(0.25 * statistic.LargestArea());
            mesh2.refine(quality, true);

            SimpleSmoother smoother = new SimpleSmoother();
            statistic.update((Mesh) mesh2, 10);
            smoother.smooth(mesh2);
            System.out.println("Success");

            assert true;

        } catch (Exception e) {
            e.printStackTrace();
            assert false;
        }

        assert true;
    }
}
