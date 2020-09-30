import org.junit.Test;
import triangle.*;
import triangle.io.FileProcessor;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

public class DwyerTest {

    @Test
    public void dwyer() {
        try {
            var options = new ConstraintOptions();
            var quality = new QualityOptions();

            options.setConformingDelaunay(true);
            quality.setMinAngle(20);
            quality.setMaxAngle(100);

            // Reading polygon from file
            IPolygon polygon = FileProcessor.importPolygon("face.poly");
            assertNotNull(polygon);

            // Meshing
            IMesh mesh = PolygonHelper.triangulate(polygon, options, quality);
            assertNotNull(mesh);

            // Refine a couple times
            Statistic statistic = new Statistic();
            int refines = 5;

            for (int i = 0; i < refines; i++) {
                statistic.update((Mesh) mesh, 10);
                quality.setMaxArea(0.25 * statistic.LargestArea());
                mesh.refine(quality, true);
            }

            // Smooth
            SimpleSmoother smoother = new SimpleSmoother();
            statistic.update((Mesh) mesh, 10);
            smoother.smooth(mesh);
            System.out.println("Success");

        } catch (IOException e) {
            e.printStackTrace();
            assert false;
        }


        assert true;
    }
}
