package triangle.tools;

public class MutableInt {

    private int value;

    public MutableInt(int value) {
        this.value = value;
    }

    public void increment() {
        value++;
    }

    public void decrement() {
        value--;
    }

    public MutableInt incrementAndGet() {
        value++;
        return this;
    }

    public MutableInt decrementAndGet() {
        value--;
        return this;
    }

    public void add(int value) {
        this.value += value;
    }

    public int getValue() {
        return this.value;
    }

    public void setValue(int value) {
        this.value = value;
    }
}
