package frc.robot.TaskList;

import java.util.ArrayList;

public class TaskList extends Task {
    private ArrayList<Task> tasks;

    public TaskList(Task... tasks) {
        super(() -> true);
        this.tasks = new ArrayList<Task>();
        for (int i = 0; i < tasks.length; i++) {
            this.tasks.add(tasks[i]);
        }
    }

    public void addTask(Task t) {
        this.tasks.add(t);
    }

    public int size() {
        return this.tasks.size();
    }

    public void skip() {
        this.tasks.remove(0);
    }

    public boolean isDone() {
        return this.tasks.size() == 0;
    }

    @Override
    public boolean execute() {
        if (this.tasks.size() > 0) {
            if (this.tasks.get(0).execute()) {
                this.tasks.remove(0);
            }
            return false;
        }
        return true;
    }

    public void clear() {
        while (this.tasks.size() > 0) {
            this.tasks.remove(0);
        }
    }
}