package frc.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * A Request which takes a list of Requests and executes them in parallel.
 */
public class ParallelRequest extends Request {

    List<Request> requests;

    public ParallelRequest(Request... requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    public ParallelRequest(List<Request> requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    @Override
    public void act() {
        for(Request request : requests) {
            request.act();
        }
    }

    @Override
    public boolean isFinished() {
        requests.removeIf((r) -> r.isFinished());
        return requests.isEmpty();
    }
}