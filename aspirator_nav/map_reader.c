#include <stdio.h>
#include <stdlib.h>

#define MAX_FILENAME_LENGTH 100
#define MAX_MATRIX_SIZE 1000

int main() {
    FILE *file;
    char filename[MAX_FILENAME_LENGTH];
    char magic_number[3];
    int width, height, max_value, i, j;
    int matrix[MAX_MATRIX_SIZE][MAX_MATRIX_SIZE]; // Assuming maximum matrix size

    // Prompt user for filename
    printf("Enter the .pgm file name: ");
    scanf("%s", filename);

    // Open file
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file %s\n", filename);
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
    printf("Matrix:\n");
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            printf("%d ", matrix[i][j]);
        }
        printf("\n");
    }

    return 0;
}
