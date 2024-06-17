#include <stdio.h>
#include <stdlib.h>
#include <yaml.h>
#include <math.h>

#define MAX_FILENAME_LENGTH 100
#define MAX_MATRIX_SIZE 1000
#define ROBOT_HEIGHT 0.22 // in meters 
#define ROBOT_LENGTH 0.265 // in meters

int main() {
    FILE *file;
    char filename[MAX_FILENAME_LENGTH];
    int abs_goals_x[1000];
    int abs_goals_y[1000];
    int rel_goals_x[1000];
    int rel_goals_y[1000];
    char magic_number[3];
    int width, height, max_value, i, j;
    int matrix[MAX_MATRIX_SIZE][MAX_MATRIX_SIZE]; // Assuming maximum matrix size
    unsigned char acc_map[MAX_MATRIX_SIZE][MAX_MATRIX_SIZE] = { 0 };

    // Prompt user for filename
    //printf("Enter the .pgm file name: ");
    //scanf("%s", filename);

    /// START READING YAML /// 

    FILE *yaml_file = fopen("map.yaml", "r");
    if (!yaml_file) {
        fprintf(stderr, "Failed to open file.\n");
        return 1;
    }

    char pgm_map_directory[500];
    float resolution;
    float origin_x, origin_y, origin_z;
    int negate;
    float occupied_thresh, free_thresh;

    printf("map.yaml contents: \n");

    fscanf(yaml_file, "image: %s\n", pgm_map_directory);
    printf("Directory: %s\n", pgm_map_directory);

    fscanf(yaml_file, "resolution: %f\n", &resolution);
    printf("Resolution: %f\n", resolution);

    fscanf(yaml_file, "origin: [%f, %f, %f]\n", &origin_x, &origin_y, &origin_z);
    printf("Origin_x: %f\n", origin_x);
    printf("Origin_y: %f\n", origin_y);
    printf("Origin_z: %f\n", origin_z);

    fscanf(yaml_file, "negate: %d\n", &negate);
    printf("Negate: %d\n", negate);

    fscanf(yaml_file, "occupied_thresh: %f\n", &occupied_thresh);
    printf("Occupied thresh: %f\n", occupied_thresh);

    fscanf(yaml_file, "free_thresh: %f\n", &free_thresh);
    printf("Free thresh: %f\n", free_thresh);

    fclose(yaml_file);

    printf("\n");

    /// FINISH READING YAML /// 

    // Open map.pgm file
    file = fopen("map.pgm", "r");
    if (file == NULL) {
        printf("Error: Unable to open file %s\n", "map.pgm");
        return 1;
    }

    // Read magic number
    fscanf(file, "%s\n", magic_number);
    if (magic_number[0] != 'P' || magic_number[1] != '5') {
        printf("Error: Invalid PGM file format\n");
        fclose(file);
        return 1;
    }

    // Jump comment written by creator (maybe not the best solution - consider jumping all lines that start with #)
    fscanf(file, "%*[^\n]");

    // Read width, height, and max value
    fscanf(file, "%d %d\n", &width, &height);
    fscanf(file, "%d\n", &max_value);

    printf("map.pgm metadata: \n");

    printf("Width: %d. Height: %d. Max value: %d.", width, height, max_value);
    
    // Read pixel values into matrix
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            int value = fgetc(file);
            if (value == EOF) {
                printf("Error: Premature end of file\n");
                fclose(file);
                return 1;
            }
            matrix[i][j] = value;
        }
    }

    // Close file
    fclose(file);

    // Print matrix
    /*printf("Matrix:\n");
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            printf("%d ", matrix[i][j]);
        }
        printf("\n");
    }*/

    float robot_radius_m = sqrt((ROBOT_HEIGHT*ROBOT_HEIGHT + ROBOT_LENGTH*ROBOT_LENGTH) / 4); // Radius of robot in pixels
    float robot_radius_p = robot_radius_m / resolution; // Radius of robot in pixels XXXXXXX

    printf("\nrobot_radius_m = %f\nrobot_radius_p = %f\n", robot_radius_m, robot_radius_p);

    int steps_w = width / (robot_radius_p*2); // Number of horizontal steps through matrix
    int steps_h = height / (robot_radius_p*2); // Number of vertical steps through matrix

    printf("steps_w = %d\nsteps_h = %d", steps_w, steps_h);

    // Fill matrix with naviagable points
    int k, l;
    int unac = 0; // Unnaccessible region
    for (i = 0; i < steps_w; i++) {  
        for (j = 0; j < steps_h; j++) {
            unac = 0;
            for (k = 0; k < (int)robot_radius_p*2 && !unac; k++) {
                for (l = 0; l < (int)robot_radius_p*2 && !unac; l++) {
                    //if (k == 6 && l != 0) printf("i = %d. j = %d. k = %d. l = %d. \n", i, j, k, l);
                    //printf("Vou olhar matrix[%d][%d]. O valor eh %d.  ", i * (int)robot_radius_p*2 + k, j * (int)robot_radius_p*2 + l, matrix[i * (int)robot_radius_p*2 + k][j * (int)robot_radius_p*2 + l]);
                    if (matrix[i * (int)robot_radius_p*2 + k][j * (int)robot_radius_p*2 + l] < 240) { // se achou ponto preto ou cinza
                        unac = 1;
                    }
                    if (k == (int)robot_radius_p*2 - 1 && l == (int)robot_radius_p*2 - 1 && !unac) {
                        acc_map[i * (int)robot_radius_p*2 + k/2][j * (int)robot_radius_p*2 + l/2] = 254;
                    }
                }
            }
        }
    }

    printf("\nExporting images.\n");

    // Export matrix as image to verify
    FILE *nav_map;
    nav_map = fopen("nav_map.pgm", "w");

    fprintf(nav_map, "P2\n");

    fprintf(nav_map, "# CREATOR: map_reader.c %.2f m/pix\n", resolution);

    fprintf(nav_map, "%d %d\n", width, height);

    fprintf(nav_map, "%d\n", max_value);

    for (i = 0; i < width; i++) {
        for (j = 0; j < height; j++) {
            fprintf(nav_map, "%d ", acc_map[i][j]);
        }
    }

    fclose(nav_map);

    // Exporting the overlayed map
    FILE *ov_map;
    ov_map = fopen("ov_map.ppm", "w");

    fprintf(ov_map, "P3\n");

    fprintf(ov_map, "# CREATOR: map_reader.c %.2f m/pix\n", resolution);

    fprintf(ov_map, "%d %d\n", width, height);

    fprintf(ov_map, "255\n");

    int nav_points_counter = 0; // Contador de pontos navegaveis
    for (i = 0; i < width; i++) {
        for (j = 0; j < height; j++) {
            if (acc_map[i][j] == 0) {
                fprintf(ov_map, "%d %d %d ", matrix[i][j], matrix[i][j], matrix[i][j]);
            }
            else {
                fprintf(ov_map, "0 255 0 ");
                abs_goals_x[nav_points_counter] = i;
                abs_goals_y[nav_points_counter] = j;
                nav_points_counter++;
            }
        fprintf(ov_map, "\n");
        }
    }

    for (i = 0; i < nav_points_counter; i++) {
        //printf("goals_x[%d]: %d\n", i, abs_goals_x[i]);
        //printf("goals_y[%d]: %d\n", i, abs_goals_y[i]);
        rel_goals_x[i] = abs_goals_x[i] - origin_x;
        rel_goals_y[i] = abs_goals_y[i] - origin_y;
        //printf("rel_goals_x[%d]: %d\n", i, rel_goals_x[i]);
        //printf("rel_goals_y[%d]: %d\n", i, rel_goals_y[i]);
    }

    fclose(ov_map);

    return 0;
}
