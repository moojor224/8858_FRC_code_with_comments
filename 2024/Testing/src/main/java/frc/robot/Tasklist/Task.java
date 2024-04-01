package frc.robot.TaskList;

import java.util.function.Supplier;

public class Task {
    private Supplier<Boolean> periodic; // the task's main function that runs periodicaly
    private Runnable init = () -> { // an init function that runs once the first time that the task is run
    };
    private boolean initialized = false; // whether the init function has been run yet

    public Task(Supplier<Boolean> periodic) {
        this.periodic = periodic;
    }

    public Task(Runnable init, Supplier<Boolean> periodic) {
        this.init = init;
        this.periodic = periodic;
    }

    public boolean execute() { // runs the task
        if (!this.initialized) {
            this.init.run();
            this.initialized = true;
        }
        return this.periodic.get(); // returns true if the task is complete
    }
}