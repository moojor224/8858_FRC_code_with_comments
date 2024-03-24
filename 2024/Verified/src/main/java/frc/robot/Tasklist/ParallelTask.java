package frc.robot.TaskList;

import java.util.ArrayList;

public class ParallelTask extends Task {
    private ArrayList<Task> tasks;

    public ParallelTask(Task... tasks) {
        super(() -> false);
        this.tasks = new ArrayList<Task>();
        for (int i = 0; i < tasks.length; i++) {
            this.tasks.add(tasks[i]);
        }
    }

    @Override
    public boolean execute() { // runs the tasks
        boolean done = true;
        for (int i = 0; i < this.tasks.size(); i++) {
            if (!this.tasks.get(i).execute()) {
                done = false;
            }
        }
        return done;
    }
}
