package triangle.smoothing;

import triangle.IMesh;

public interface ISmoother {

    void smooth(IMesh mesh);

    void smooth(IMesh mesh, int limit);
}
