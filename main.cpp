#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <SFML/Graphics.hpp>

using namespace sf;

const int GRID_SIZE_Y = 120; // number of cell in the Y direction
const int GRID_SIZE_X = 100; // number of cell in the X direction
const int CELL_SIZE = 10; // Cell size

const float TIME_STEP = 0.01;
const float OVERELAXATION = 1.9;
const int ITERATION = 1000;
const int PROJECTION_ITERATIONS = 40; // Gauss-Seidel sweeps per frame
const float INFLOW_SPEED = 110; // horizontal speed injected at the left source column
const float OBSTACLE_RADIUS = 8; // radius (in cells) of the draggable obstacle

// Obstacle shapes, cycled with the Space key
const int SHAPE_CIRCLE = 0;
const int SHAPE_SQUARE = 1;
const int SHAPE_PLATE = 2;
const int SHAPE_COUNT = 3;

// Initailisation of the grid
void InitGrid(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y]);

// Initialisation of the environment
void InitEnvironment(float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Initialisation of the fluid 
// is it necessary to mark the box with boundary conditions ?
// Initialisation of the known velocities
// Initialisation of the known concentrations
void InitFluid(float Fluid[GRID_SIZE_X][GRID_SIZE_Y][3]);

// Update the Grid
void UpdateGrid(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y], float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Display Simulation
void DisplaySimu(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y], RenderWindow& window);

// Make sure that the divergence of the velocity is null for each box
void Incompressibility(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Update the velocity field by self-advection (Semi-Lagrangian)
void VelocityAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], const float prevFluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Calculate the concentration of the smoke by and interpolation
void SmokeAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], const float prevFluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Interpolation of the concentration (Semi-Lagrangian)
float Interpolation(float c1, float c2, float c3, float c4, float x, float y);

// Re-impose the fixed inflow velocity at the left source column
void ResetInflow(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Place/move the obstacle, imparting its velocity into the fluid it displaces
void SetObstacle(float environment[GRID_SIZE_X][GRID_SIZE_Y], float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float centerI, float centerJ, float radius, float velI, float velJ, bool moving, int shape);

int main()
{

    // Environment 
    // if <1> fluid 
    // if <0> obstacle
    float Environment[GRID_SIZE_X][GRID_SIZE_Y];

    // Fluid
    // <1> Horizontal speed of the cell (U)
    // <2> Vertical speed of the cell (V)
    // <3> Concentration between 0 and 1
    float Fluid[GRID_SIZE_X][GRID_SIZE_Y][3];

    // Snapshot of Fluid from before the current advection pass, so advection
    // always reads a consistent, unmodified field instead of partially-updated cells
    float FluidPrev[GRID_SIZE_X][GRID_SIZE_Y][3];

    // Create the grid
    VertexArray Grid[GRID_SIZE_X][GRID_SIZE_Y];

    // Create a window type RenderWindow
    RenderWindow SimuWindow(VideoMode({GRID_SIZE_Y * CELL_SIZE, GRID_SIZE_X * CELL_SIZE}), "Fluid Simulation");

    SimuWindow.clear(); // Clear the window
    InitFluid(Fluid); // Initilaize the initial conditions od the fluid
    InitEnvironment(Environment); // Inialize the obstacle
    InitGrid(Grid); // Create the grid with all the cells

    // Draggable obstacle, in grid coordinates (i = row/X, j = column/Y)
    float obstacleI = GRID_SIZE_X * 0.5f;
    float obstacleJ = GRID_SIZE_Y * 0.33f;
    float lastMouseI = obstacleI;
    float lastMouseJ = obstacleJ;
    bool dragging = false;
    int obstacleShape = SHAPE_CIRCLE;
    SetObstacle(Environment, Fluid, obstacleI, obstacleJ, OBSTACLE_RADIUS, 0, 0, false, obstacleShape);

    UpdateGrid(Grid, Fluid, Environment); // Updtae the grid before displaying
    DisplaySimu(Grid, SimuWindow); // Display the Simulation

    for (int i = 0; i < ITERATION; i++) {
        Incompressibility(Fluid, Environment);
    }

    int frameCount = 0;

    while (SimuWindow.isOpen()) {
        while (auto event = SimuWindow.pollEvent()) {
            // Close the window if we close the window
            if (event->is<Event::Closed>()) {
                SimuWindow.close(); // close the window
            }

            // Click anywhere to grab the obstacle and teleport it there
            if (const auto* pressed = event->getIf<Event::MouseButtonPressed>()) {
                if (pressed->button == Mouse::Button::Left) {
                    lastMouseI = pressed->position.y / (float)CELL_SIZE;
                    lastMouseJ = pressed->position.x / (float)CELL_SIZE;
                    obstacleI = lastMouseI;
                    obstacleJ = lastMouseJ;
                    dragging = true;
                    SetObstacle(Environment, Fluid, obstacleI, obstacleJ, OBSTACLE_RADIUS, 0, 0, false, obstacleShape);
                }
            }

            // Drag the obstacle around, imparting its velocity into the fluid
            if (const auto* moved = event->getIf<Event::MouseMoved>()) {
                if (dragging) {
                    float mouseI = moved->position.y / (float)CELL_SIZE;
                    float mouseJ = moved->position.x / (float)CELL_SIZE;
                    float velI = (mouseI - lastMouseI) / TIME_STEP;
                    float velJ = (mouseJ - lastMouseJ) / TIME_STEP;
                    obstacleI = mouseI;
                    obstacleJ = mouseJ;
                    lastMouseI = mouseI;
                    lastMouseJ = mouseJ;
                    SetObstacle(Environment, Fluid, obstacleI, obstacleJ, OBSTACLE_RADIUS, velI, velJ, true, obstacleShape);
                }
            }

            if (const auto* released = event->getIf<Event::MouseButtonReleased>()) {
                if (released->button == Mouse::Button::Left) {
                    dragging = false;
                }
            }

            // Space cycles the obstacle through circle / square / flat plate
            if (const auto* key = event->getIf<Event::KeyPressed>()) {
                if (key->code == Keyboard::Key::Space) {
                    obstacleShape = (obstacleShape + 1) % SHAPE_COUNT;
                    SetObstacle(Environment, Fluid, obstacleI, obstacleJ, OBSTACLE_RADIUS, 0, 0, false, obstacleShape);
                }
            }
        }

        for (int iter = 0; iter < PROJECTION_ITERATIONS; iter++) {
            Incompressibility(Fluid, Environment);
        }
        ResetInflow(Fluid, Environment);

        std::memcpy(FluidPrev, Fluid, sizeof(Fluid));
        VelocityAdvection(Fluid, FluidPrev, Environment);

        std::memcpy(FluidPrev, Fluid, sizeof(Fluid));
        SmokeAdvection(Fluid, FluidPrev, Environment);

        // Reseed alternating dye stripes at the source so the airflow stays
        // visible as streaklines instead of a single blob that fades away
        frameCount++;
        bool stripeOn = (frameCount / 15) % 2 == 0;
        for (int i = 35; i <= 63; i++) {
            if (Environment[i][1] == 1) {
                Fluid[i][1][2] = stripeOn ? 1.0f : 0.0f;
            }
        }

        SimuWindow.clear();
        UpdateGrid(Grid, Fluid, Environment);
        DisplaySimu(Grid, SimuWindow);
    }

    return 0;
}

// Make sure that the divergence of the velocity is null for each cell 
void Incompressibility(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {
    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {

            if (environment[i][j] == 1) {// if there is a fluid on the cell

                // number of cell with fluid around the cell[i][j]
                int numCell = environment[i][j - 1] + environment[i][j + 1] + environment[i - 1][j] + environment[i + 1][j];

                if (numCell == 0) continue; // fully enclosed by obstacles, nothing to redistribute

                // calculate the divergence
                // take account of the overrelaxation ?
                float div = OVERELAXATION * (fluid[i][j][0] - fluid[i][j + 1][0] + fluid[i][j][1] - fluid[i + 1][j][1]);

                fluid[i][j][0] = fluid[i][j][0] - ((div * environment[i][j - 1]) / numCell); // horizontal speed from the left of the cell
                fluid[i][j + 1][0] = fluid[i][j + 1][0] + ((div * environment[i][j + 1]) / numCell); // horizontal speed at the right of the cell
                fluid[i][j][1] = fluid[i][j][1] - ((div * environment[i - 1][j]) / numCell); // vertical speed from the top of the cell
                fluid[i + 1][j][1] = fluid[i + 1][j][1] + ((div * environment[i + 1][j]) / numCell); // vertical speed at the bottom of the cell
            }
        }
    } 
}

// Update the velocity field by self-advection (Semi-Lagrangian)
void VelocityAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], const float prevFluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {

    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {

            if (environment[i][j] != 1) continue; // leave obstacle-forced velocities untouched

            // Horizontal speed U, stored at the left face of cell [i][j]
            float vAtU = (prevFluid[i][j][1] + prevFluid[i + 1][j][1] + prevFluid[i][j - 1][1] + prevFluid[i + 1][j - 1][1]) * 0.25f;
            float xU = i - TIME_STEP * vAtU;
            float yU = j - TIME_STEP * prevFluid[i][j][0];
            xU = std::clamp(xU, 1.0f, (float)(GRID_SIZE_X - 2));
            yU = std::clamp(yU, 1.0f, (float)(GRID_SIZE_Y - 2));
            int CellXu = xU;
            int CellYu = yU;
            xU -= CellXu;
            yU -= CellYu;
            fluid[i][j][0] = Interpolation(prevFluid[CellXu][CellYu][0], prevFluid[CellXu][CellYu + 1][0], prevFluid[CellXu + 1][CellYu][0], prevFluid[CellXu + 1][CellYu + 1][0], xU, yU);

            // Vertical speed V, stored at the top face of cell [i][j]
            float uAtV = (prevFluid[i][j][0] + prevFluid[i][j + 1][0] + prevFluid[i - 1][j][0] + prevFluid[i - 1][j + 1][0]) * 0.25f;
            float xV = i - TIME_STEP * prevFluid[i][j][1];
            float yV = j - TIME_STEP * uAtV;
            xV = std::clamp(xV, 1.0f, (float)(GRID_SIZE_X - 2));
            yV = std::clamp(yV, 1.0f, (float)(GRID_SIZE_Y - 2));
            int CellXv = xV;
            int CellYv = yV;
            xV -= CellXv;
            yV -= CellYv;
            fluid[i][j][1] = Interpolation(prevFluid[CellXv][CellYv][1], prevFluid[CellXv][CellYv + 1][1], prevFluid[CellXv + 1][CellYv][1], prevFluid[CellXv + 1][CellYv + 1][1], xV, yV);
        }
    }
}

// Calculate the concentration of the smoke by and interpolation
void SmokeAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], const float prevFluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {

    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {

            if (environment[i][j] == 1) { // only advect dye inside the fluid domain

                float VSpeed = (prevFluid[i][j][1] + prevFluid[i + 1][j][1]) * 0.5;
                float USpeed = (prevFluid[i][j][0] + prevFluid[i][j + 1][0]) * 0.5;

                float x = i - TIME_STEP * VSpeed;
                float y = j - TIME_STEP * USpeed;

                // Keep the backtraced point inside the grid instead of zeroing the cell out
                x = std::clamp(x, 1.0f, (float)(GRID_SIZE_X - 2));
                y = std::clamp(y, 1.0f, (float)(GRID_SIZE_Y - 2));

                int CellX = x; // Integer of x
                int CellY = y; // Integer of y

                x = x - CellX; // rest
                y = y - CellY; // rest

                fluid[i][j][2] = Interpolation(prevFluid[CellX][CellY][2], prevFluid[CellX][CellY + 1][2], prevFluid[CellX + 1][CellY][2], prevFluid[CellX + 1][CellY + 1][2], x, y);
            }
        }
    }
}

// Re-impose the fixed inflow velocity at the left source column
void ResetInflow(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {
    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        if (environment[i][1] == 1) {
            fluid[i][1][0] = INFLOW_SPEED;
        }
    }
}

// Place/move the obstacle, imparting its velocity into the fluid it displaces
void SetObstacle(float environment[GRID_SIZE_X][GRID_SIZE_Y], float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float centerI, float centerJ, float radius, float velI, float velJ, bool moving, int shape) {

    // Rebuild the interior as all-fluid, then carve the obstacle shape out of it
    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {
            environment[i][j] = 1;
        }
    }

    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {
            float di = i - centerI;
            float dj = j - centerJ;

            bool inside = false;
            if (shape == SHAPE_SQUARE) {
                // Square block, same footprint (2*radius side) as the circle's diameter
                inside = (std::abs(di) <= radius) && (std::abs(dj) <= radius);
            } else if (shape == SHAPE_PLATE) {
                // Thin flat plate held face-on to the flow, tall and narrow, to trigger stronger vortex shedding
                inside = (std::abs(dj) <= radius * 0.15f) && (std::abs(di) <= radius * 1.5f);
            } else {
                // Circle (default)
                inside = di * di + dj * dj <= radius * radius;
            }

            if (inside) {
                environment[i][j] = 0;
                if (moving) {
                    fluid[i][j][0] = velI;
                    fluid[i][j][1] = velJ;
                }
            }
        }
    }
}

// Interpolation of the concentration (Semi-Lagrangian)
float Interpolation(float c1, float c2, float c3, float c4, float x, float y) {
    
    float C = (c1 * (1 - x) * (1 - y)) + (c2 * (1 - x) * y) + (c3 * x * (1 - y)) + (c4 * x * y);
    return C;
}


// Create the grid where the simulation is display
void InitGrid(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y]) {

    for (int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {

            grid[i][j].setPrimitiveType(PrimitiveType::TriangleFan);
            grid[i][j].resize(4);

            grid[i][j][0].position = Vector2f(j * CELL_SIZE, i * CELL_SIZE);
            grid[i][j][1].position = Vector2f(j * CELL_SIZE, (i + 1) * CELL_SIZE);
            grid[i][j][2].position = Vector2f((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE);
            grid[i][j][3].position = Vector2f((j + 1) * CELL_SIZE, i * CELL_SIZE);
            
        }
    }
}

// Initialisation of the environment
void InitEnvironment(float environment[GRID_SIZE_X][GRID_SIZE_Y]) {
    
    // initialialise the environment
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            environment[i][j] = 0;
        }
    }

    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {
            environment[i][j] = 1;
        }
    }
    // The obstacle is carved out separately by SetObstacle(), called once at
    // startup and again whenever the user drags it.
}

void InitFluid(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3]) {

    // put 0 on all the table 
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            for (int k = 0; k < 3; k++) {
                fluid[i][j][k] = 0;
            }
        }
    }
    
    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        fluid[i][1][0] = 110;
    }

    for (int i = 35; i <= 63; i++) {
        fluid[i][1][2] = 1;
    }
}

// Update the Grid
// at each step the grid as to be updated before display
void UpdateGrid(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y], float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {
    
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {

            if (environment[i][j] == 1) {

                int GrayScale = fluid[i][j][2] * 255;
                for (int k = 0; k < 4; k++) {
                    grid[i][j][k].color = Color(GrayScale, GrayScale, GrayScale); // gray scale
                }

            }else {

                for (int k = 0; k < 4; k++) {
                    grid[i][j][k].color = Color(255, 0, 0); // red
                }
            }
        }
    }
}

// Display Simulation
void DisplaySimu(VertexArray grid[GRID_SIZE_X][GRID_SIZE_Y], RenderWindow& window) {
    
    window.clear();

    // Dessiner la grille
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            window.draw(grid[i][j]);
        }
    }
    window.display();
}


