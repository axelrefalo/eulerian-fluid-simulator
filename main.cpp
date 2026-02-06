#include <iostream>
#include <SFML/Graphics.hpp>

using namespace sf;

const int GRID_SIZE_Y = 120; // number of cell in the Y direction 
const int GRID_SIZE_X = 100; // number of cell in the X direction 
const int CELL_SIZE = 10; // Cell size

const float TIME_STEP = 0.01;
const float OVERELAXATION = 1;
const int ITERATION = 1000;

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

// Calculate the concentration of the smoke by and interpolation 
void SmokeAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]);

// Interpolation of the concentration (Semi-Lagrangian)
float Interpolation(float c1, float c2, float c3, float c4, float x, float y);

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

    // Create the grid
    VertexArray Grid[GRID_SIZE_X][GRID_SIZE_Y];

    // Create a window type RenderWindow
    RenderWindow SimuWindow(VideoMode({GRID_SIZE_Y * CELL_SIZE, GRID_SIZE_X * CELL_SIZE}), "Fluid Simulation");
    
    SimuWindow.clear(); // Clear the window 
    InitFluid(Fluid); // Initilaize the initial conditions od the fluid
    InitEnvironment(Environment); // Inialize the obstacle
    InitGrid(Grid); // Create the grid with all the cells 
    UpdateGrid(Grid, Fluid, Environment); // Updtae the grid before displaying
    DisplaySimu(Grid, SimuWindow); // Display the Simulation 
    
    for (int i = 0; i < ITERATION; i++) {
        Incompressibility(Fluid, Environment);
    }

    while (SimuWindow.isOpen()) {
        while (auto event = SimuWindow.pollEvent()) {
            // Close the window if we close the window
            if (event->is<Event::Closed>()) {
                SimuWindow.close(); // close the window
            }
        }
        
        Incompressibility(Fluid, Environment);
        SmokeAdvection(Fluid, Environment);
     
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

                // calculate the divergence
                // take account of the overrelaxation ?
                float div = OVERELAXATION * (fluid[i][j][0] - fluid[i][j + 1][0] + fluid[i][j][1] - fluid[i + 1][j][1]); 
                
                // number of cell with fluid around the cell[i][j]
                int numCell = environment[i][j - 1] + environment[i][j + 1] + environment[i - 1][j] + environment[i + 1][j];

                fluid[i][j][0] = fluid[i][j][0] - ((div * environment[i][j - 1]) / numCell); // horizontal speed from the left of the cell
                fluid[i][j + 1][0] = fluid[i][j + 1][0] + ((div * environment[i][j + 1]) / numCell); // horizontal speed at the right of the cell
                fluid[i][j][1] = fluid[i][j][1] - ((div * environment[i - 1][j]) / numCell); // vertical speed from the top of the cell
                fluid[i + 1][j][1] = fluid[i + 1][j][1] + ((div * environment[i + 1][j]) / numCell); // vertical speed at the bottom of the cell
            }
        }
    } 
}

// Calculate the concentration of the smoke by and interpolation 
void SmokeAdvection(float fluid[GRID_SIZE_X][GRID_SIZE_Y][3], float environment[GRID_SIZE_X][GRID_SIZE_Y]) {

    for (int i = 1; i < GRID_SIZE_X - 1; i++) {
        for (int j = 1; j < GRID_SIZE_Y - 1; j++) {

            if (fluid[i][j][2] != 1) { // if it is not a boundary condition
                
                float VSpeed = (fluid[i][j][1] + fluid[i + 1][j][1]) * 0.5;
                float USpeed = (fluid[i][j][0] + fluid[i][j + 1][0]) * 0.5;

                float x = i - TIME_STEP * VSpeed;
                float y = j - TIME_STEP * USpeed;

                // Check if x and y are in the grid 
                if (x > 0 && y > 0) {

                    int CellX = x; // Integer of x 
                    int CellY = y; // Integer of y 

                    x = x - CellX; // rest
                    y = y - CellY; // rest 

                    fluid[i][j][2] = Interpolation(fluid[CellX][CellY][2], fluid[CellX][CellY + 1][2], fluid[CellX + 1][CellY][2], fluid[CellX + 1][CellY + 1][2], x, y);
                }
                else {
                    // Nothing is outside the window
                    fluid[i][j][2] = 0;
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
    
    // draw walls
    for (int c1 = 0; c1 < 10; c1++) {
        for (int c2 = 0; c2 < 10; c2++) {

        }
    }
    for (int i = 24; i <= 74; i++) {
        for (int j = 60; j < 65; j++) {
            environment[i][j] = 0;
        }
    }
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


