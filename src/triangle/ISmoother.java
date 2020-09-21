package triangle;

public interface ISmoother {

    void smooth(IMesh mesh);

    void smooth(IMesh mesh, int limit);
}
