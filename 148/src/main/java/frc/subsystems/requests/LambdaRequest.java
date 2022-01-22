package frc.subsystems.requests;

public class LambdaRequest extends Request{

    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public LambdaRequest(VoidInterface f) {
        mF = f;
    }

    @Override
    public void act() {
        mF.f();
    }
}