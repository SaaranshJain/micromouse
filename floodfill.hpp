#ifndef FLOODFILL_H
#define FLOODFILL_H

enum ActionType {
    GoForward,
    TurnRight,
    TurnLeft,
};

struct Action {
    ActionType type;
    int value;
};

typedef struct Action Action;

class Maze {
    public:
    int* matrix;

    Maze(char filename[50]);
    static void printmaze(int* matrix);
};

class Micromouse {
    public:
    int* mapped_matrix;
    unsigned short curr_x;
    unsigned short curr_y;
    char x_dir;
    char y_dir;

    Micromouse();
    void place_mouse(unsigned short x, unsigned short y);
    bool go_forward(Maze maze, unsigned short x, unsigned short y);
    void set_dir(char x_dir, char y_dir);
    std::vector<std::pair<int, int>> navigate_maze(Maze maze, bool to_goal);
    void floodfill(unsigned short goal_x, unsigned short goal_y);
};

#endif
