#ifndef FLOODFILL_H
#define FLOODFILL_H

enum ActionType {
    GoForward,
    TurnRight,
    TurnLeft,
    TurnAround
};

struct Action {
    ActionType type;
    int value;
};

typedef struct Action Action;

void printmaze(int* matrix);

class Micromouse {
    public:
    int* mapped_matrix;
    unsigned short curr_x;
    unsigned short curr_y;
    char x_dir;
    char y_dir;

    Micromouse();
    bool go_forward(double distanceF, double distanceL, double distanceR, unsigned short goal_x, unsigned short goal_y);
    void set_dir(char x_dir, char y_dir);
    void turnRight();
    void turnLeft();
    void turnAround();
    std::vector<std::pair<int, int>> navigate_maze(Maze maze, bool to_goal);
    void floodfill(unsigned short goal_x, unsigned short goal_y);
};

#endif
